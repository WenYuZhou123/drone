#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>
#include <csignal>
#include <cstdlib>
#include <cv_bridge/cv_bridge.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <iomanip>
#include <iostream>
#include <lib_library.h>
#include <math.h>
#include <mavros/frame_tf.h>
#include <mavros_msgs/ADSBVehicle.h>
#include <mavros_msgs/WaypointList.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdio.h>
#include <string>
#include <vector>
#include <yolov8_ros_msgs/BoundingBoxes.h>

#define ALTITUDE 3.0

geographic_msgs::GeoPointStamped target_glo_pos_msg;
mavros_msgs::PositionTarget setpoint_raw;

sensor_msgs::NavSatFix glo_pos_msg;
sensor_msgs::NavSatFix gps_udp_msg;
Eigen::Vector3d current_local_pos;

bool glo_pos_cb_flag = false;

class OBJ_POS
{
  public:
    OBJ_POS()
    {
        name = "no object";
        x = 0;
        y = 0;
    }

    string name;
    int64 x;
    int64 y;
};

std::vector<OBJ_POS> obj_pos;
OBJ_POS obj_pos_;

std::vector<geometry_msgs::PoseStamped> pose;

yolov8_ros_msgs::BoundingBoxes object_tmp;

ros::Subscriber gps_sub;

ros::Subscriber waypoint_sub;

ros::Subscriber local_pos_pos_sub;

ros::Subscriber state_sub;

ros::Subscriber local_pos_sub;

ros::Subscriber glo_pos_sub;

ros::Subscriber Object_sub;

ros::Publisher mavros_adsb_pub;

ros::Publisher gps_udp_pub;

ros::Publisher setpoint_raw_pub;

ros::ServiceClient arming_client;

ros::ServiceClient set_mode_client;

ros::ServiceClient ctrl_pwm_client;

float uav_altitude = 3;
ros::Publisher target_glo_pos_pub;
void glo_pos_cb(const sensor_msgs::NavSatFix msg);
void glo_pos_cb(const sensor_msgs::NavSatFix msg)
{
    glo_pos_msg = msg;
}

nav_msgs::Odometry local_pos;
double init_position_x_take_off = 0.0;
double init_position_y_take_off = 0.0;
double init_position_z_take_off = 0.0;
double init_yaw_take_off = 0.0;
tf::Quaternion quat;
double roll = 0.0, pitch = 0.0, yaw = 0.0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    if (flag_init_position == false && (local_pos.pose.pose.position.z != 0))
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

void local_pos_pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    current_local_pos = mavros::ftf::to_eigen(msg->pose.position);
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State msg);
void state_cb(const mavros_msgs::State msg)
{
    current_state = msg;
}

// 经纬度到米的转换比例
const double LAT_TO_M = 1.1;
const double LON_TO_M = 1.0;
const double DEGREE_TO_M = 0.00001;

// 计算两个坐标之间的距离
double calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
    double delta_lat = (lat2 - lat1) * LAT_TO_M / DEGREE_TO_M;
    double delta_lon = (lon2 - lon1) * LON_TO_M / DEGREE_TO_M;
    return std::sqrt(delta_lat * delta_lat + delta_lon * delta_lon);
}

const double ALLOW_DISTANCE = 5.0;
// 检查新坐标是否已经存在于容器中
bool isNewTarget(double new_lat, double new_lon, const std::vector<std::pair<double, double>> &targets)
{
    for (const auto &target : targets)
    {
        double lat = target.first;
        double lon = target.second;
        if (calculateDistance(lat, lon, new_lat, new_lon) < ALLOW_DISTANCE)
        {
            return false;
        }
    }

    return true;
}

bool target_fuzzy_coordinates_permission = false;
bool target_exact_coordinates_permission = false;
bool return_mode_no_publish = false;

bool break_to_recognize_permission = false;

float target_id = 1;
vector<float> target_id_vector;

int object_recognize_true = 0;
int object_recognize_false = 0;

double last_publish_time = 0;
const double PUBLISH_INTERVAL = 4.0;

vector<pair<double, double>> coordinates_vector;
pair<double, double> coordinates_pair;

vector<mavros_msgs::PositionTarget> fuzzy_object_coordinates;
geometry_msgs::PointStamped object_position;
void ObjectCallback(const yolov8_ros_msgs::BoundingBoxes &msg)
{
    object_tmp = msg;
    obj_pos.clear();
    uav_altitude = local_pos.pose.pose.position.z;

    //只识别第一个
    if (!object_tmp.bounding_boxes.empty())
    {
        obj_pos_.name = object_tmp.bounding_boxes[0].Class;
        obj_pos_.x =
            (object_tmp.bounding_boxes[0].xmin + object_tmp.bounding_boxes[0].xmax) / 2; //中心点，(Xmax + Xmin)/2
        obj_pos_.y = (object_tmp.bounding_boxes[0].ymin + object_tmp.bounding_boxes[0].ymax) / 2;
        obj_pos.push_back(obj_pos_);
        object_position.header.frame_id = object_tmp.bounding_boxes[0].Class;
        object_position.point.x = obj_pos_.x;
        object_position.point.y = obj_pos_.y;

        if (object_tmp.bounding_boxes[0].Class == "landing")
        {
            object_recognize_true++;
        }
        else
        {
            object_recognize_false++;
        }

        if (object_recognize_true + object_recognize_false >= 5)
        {
            if (object_recognize_true >= 2)
            {
                // 获取当前时间
                double current_time = ros::Time::now().toSec();

                if (current_time - last_publish_time > PUBLISH_INTERVAL)
                {
                    /*第一步：单独测试根据高度和内参计算三维作为位置
                    此处根据高度计算实际偏移距离已经验证，误差在允许范围内*/
                    float distance_x = ((obj_pos_.x - 314.2472) * uav_altitude) / 581.88585;
                    float distance_y = ((obj_pos_.y - 210.2709) * uav_altitude) / 592.27138;
                    float distance = sqrt(pow(distance_x, 2) + pow(distance_y, 2));
                    /*第二步：使用正北或者正东方向进行简单定位测试
                    此处根据角度测试偏移正常，无人机在控制方向的时候需要要控制机头朝向正北方，即ROS下的1.57弧度偏航角*/
                    double angle = fabs(atan(distance_x / distance_y));
                    double tmp_long = (obj_pos_.x - 320 >= 0) ? (1 * distance * sin(angle) * 0.00001)
                                                              : (-1 * distance * sin(angle) * 0.00001);
                    double tmp_lat = (obj_pos_.y - 240 >= 0) ? (-1 * distance * cos(angle) * 0.00001 / 1.1)
                                                             : (distance * cos(angle) * 0.00001 / 1.1);
                    /*第三步：任意角度，计算经纬度,等待验证
                    //long2 = long1 + (sqrt(pow(((obj_pos_.x-314.2472)*uav_altitude)/(581.88585),2) +
                    pow(((obj_pos_.y-210.27091)*uav_altitude)/(592.27138),2)))*sin(yaw(ned)-1.57)*0.00001
                    //lat2  = lat1  + (sqrt(pow(((obj_pos_.x-314.2472)*uav_altitude)/(581.88585),2) +
                    pow(((obj_pos_.y-210.27091)*uav_altitude)/(592.27138),2)))*cosα(yaw(ned)-1.57)/1.1)*0.00001*/
                    target_glo_pos_msg.position.latitude = glo_pos_msg.latitude + tmp_lat;
                    target_glo_pos_msg.position.longitude = glo_pos_msg.longitude + tmp_long;
                    target_glo_pos_msg.position.altitude = target_id + 19;
                    coordinates_pair = {target_glo_pos_msg.position.latitude, target_glo_pos_msg.position.longitude};

                    if (isNewTarget(target_glo_pos_msg.position.latitude, target_glo_pos_msg.position.longitude,
                                    coordinates_vector))
                    {
                        target_fuzzy_coordinates_permission = true;
                        coordinates_vector.push_back(coordinates_pair);
                        last_publish_time = current_time;
                    }

                    object_recognize_true = 0;
                    object_recognize_false = 0;
                }
            }
            else
            {
                object_recognize_true = 0;
                object_recognize_false = 0;
            }
        }
    }

    if (return_mode_no_publish == false && target_fuzzy_coordinates_permission == true)
    {
        mavros_msgs::PositionTarget tmp_setpoint;
        target_id_vector.push_back(target_glo_pos_msg.position.altitude);
        target_id++;
        target_glo_pos_pub.publish(target_glo_pos_msg);
        target_fuzzy_coordinates_permission = false;
        //发布模糊经纬度坐标，并记录下当前坐标位置
        tmp_setpoint.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        tmp_setpoint.coordinate_frame = 1;
        tmp_setpoint.position.x = local_pos.pose.pose.position.x;
        tmp_setpoint.position.y = local_pos.pose.pose.position.y;
        tmp_setpoint.position.z = init_position_z_take_off + ALTITUDE;
        tmp_setpoint.yaw = yaw;
        fuzzy_object_coordinates.push_back(tmp_setpoint);
        break_to_recognize_permission = true;
        cout << "目标模糊经纬度发布:"
             << "\tlatitude:" << target_glo_pos_msg.position.latitude
             << "\tlongitude:" << target_glo_pos_msg.position.longitude
             << "\taltitude:" << target_glo_pos_msg.position.altitude << endl;
    }

    if (target_exact_coordinates_permission == true)
    {
        target_glo_pos_msg.position.latitude = glo_pos_msg.latitude;
        target_glo_pos_msg.position.longitude = glo_pos_msg.longitude;

        if (!target_id_vector.empty())
        {
            target_glo_pos_msg.position.altitude = target_id_vector.front() + 100;
            target_id_vector.erase(target_id_vector.begin());
        }
        else
        {
            target_glo_pos_msg.position.altitude = 999;
        }

        target_glo_pos_pub.publish(target_glo_pos_msg);
        target_exact_coordinates_permission = false;
        cout << "目标精确经纬度发布:"
             << "\tlatitude:" << target_glo_pos_msg.position.latitude
             << "\tlongitude:" << target_glo_pos_msg.position.longitude
             << "\taltitude:" << target_glo_pos_msg.position.altitude << endl;
    }
}

float object_recognize_track_vel_last_time_position_x_yolo8;
float object_recognize_track_vel_last_time_position_y_yolo8;
bool object_recognize_track_vel_flag_yolo8;

double position_detec_x = 0;
double position_detec_y = 0;

bool object_recognize_track_vel_yolo8(string str, float yaw, float altitude, float speed, float error_max);
bool object_recognize_track_vel_yolo8(string str, float yaw, float altitude, float speed, float error_max)
{
    if (!object_recognize_track_vel_flag_yolo8)
    {
        object_recognize_track_vel_last_time_position_x_yolo8 = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y_yolo8 = local_pos.pose.pose.position.y;
        object_recognize_track_vel_flag_yolo8 = true;
        ROS_INFO("开始识别目标并且保持跟踪");
    }

    if (!obj_pos.empty() && object_position.header.frame_id == str)
    {
        object_recognize_track_vel_last_time_position_x_yolo8 = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y_yolo8 = local_pos.pose.pose.position.y;
        position_detec_x = object_position.point.x;
        position_detec_y = object_position.point.y;

        if (fabs(position_detec_x - 320) < error_max && fabs(position_detec_y - 240) < error_max)
        {
            ROS_INFO_THROTTLE(1, "目标位置在中心点附近，保持当前速度");
            return true;
        }

        setpoint_raw.velocity.y =
            (position_detec_x - 320 >= error_max) ? -speed : (position_detec_x - 320 <= -error_max) ? speed : 0;
        setpoint_raw.velocity.x =
            (position_detec_y - 240 >= error_max) ? -speed : (position_detec_y - 240 <= -error_max) ? speed : 0;
        setpoint_raw.type_mask = 1 + 2 + /* 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 + 1024 + 2048;
        setpoint_raw.coordinate_frame = 8;
        setpoint_raw.position.z = init_position_z_take_off + altitude;
    }
    else
    {
        setpoint_raw.position.x = object_recognize_track_vel_last_time_position_x_yolo8;
        setpoint_raw.position.y = object_recognize_track_vel_last_time_position_y_yolo8;
        setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
        setpoint_raw.coordinate_frame = 1;
        setpoint_raw.position.z = init_position_z_take_off + altitude;
        setpoint_raw.yaw = yaw;
    }

    return false;
}

pid_t rosbag_pid = -1;

// 启动rosbag记录
void startRosbagRecord()
{
    int ret = system("rosbag record -O /home/cwkj/cwkj_ws/src/complete_mission/bag_saved/video.bag "
                     "/yolov8/detection_image & echo $! > /home/cwkj/cwkj_ws/src/complete_mission/rosbag_pid.txt");

    if (ret == 0)
    {
        FILE *pidFile = fopen("/home/cwkj/cwkj_ws/src/complete_mission/rosbag_pid.txt", "r");

        if (pidFile != nullptr)
        {
            fscanf(pidFile, "%d", &rosbag_pid);
            fclose(pidFile);
        }
    }
}

// 停止rosbag记录
void stopRosbagRecord()
{
    if (rosbag_pid > 0)
    {
        kill(rosbag_pid, SIGTERM);
    }
}

void shutdownCallback(int signum)
{
    stopRosbagRecord();
    ros::shutdown();
}

mavros_msgs::WaypointList waypoints;
bool flag_waypoints_receive = false;
void waypoints_cb(const mavros_msgs::WaypointList::ConstPtr &msg)
{
    flag_waypoints_receive = true;
    waypoints = *msg;
}

Eigen::Vector3d current_gps;
void gps_cb(const sensor_msgs::NavSatFix::ConstPtr &msg)
{
    current_gps = {msg->latitude, msg->longitude, msg->altitude};
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "complete_mission");
    ros::NodeHandle nh;
    gps_sub = nh.subscribe("/mavros/global_position/global", 100, gps_cb);
    waypoint_sub = nh.subscribe<mavros_msgs::WaypointList>("/mavros/mission/waypoints", 100, waypoints_cb);
    state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);
    local_pos_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 100, local_pos_pos_cb);
    glo_pos_sub = nh.subscribe<sensor_msgs::NavSatFix>("/mavros/global_position/global", 100, glo_pos_cb);
    Object_sub = nh.subscribe("/yolov8/BoundingBoxes", 100, ObjectCallback);
    setpoint_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    target_glo_pos_pub = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 100);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
    gps_udp_pub = nh.advertise<sensor_msgs::NavSatFix>("/gps_udp_bridge", 100);
    ros::Rate rate(20);
    signal(SIGINT, shutdownCallback);

    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    
    gps_udp_msg.latitude = 0;
    gps_udp_msg.longitude = 0;

    //此处wihle等待上传航点，没有航点的话则一直等待
    while (ros::ok())
    {
        for (int index = 0; index < waypoints.waypoints.size(); index++)
        {
            if (waypoints.waypoints[index].command == 20)
            {
            }
            else
            {
                geometry_msgs::PoseStamped p;
                GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                                GeographicLib::Constants::WGS84_f());
                Eigen::Vector3d goal_gps(waypoints.waypoints[index].x_lat, waypoints.waypoints[index].y_long, 0);
                Eigen::Vector3d current_ecef;
                earth.Forward(current_gps.x(), current_gps.y(), current_gps.z(), current_ecef.x(), current_ecef.y(),
                              current_ecef.z());
                Eigen::Vector3d goal_ecef;
                earth.Forward(goal_gps.x(), goal_gps.y(), goal_gps.z(), goal_ecef.x(), goal_ecef.y(), goal_ecef.z());
                Eigen::Vector3d ecef_offset = goal_ecef - current_ecef;
                Eigen::Vector3d enu_offset = mavros::ftf::transform_frame_ecef_enu(ecef_offset, current_gps);
                Eigen::Affine3d sp;
                Eigen::Quaterniond q;
                q = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
                sp.translation() = current_local_pos + enu_offset;
                sp.linear() = q.toRotationMatrix();
                //*******************************往vector容器存数据****************************************
                Eigen::Vector3d testv(sp.translation());
                p.pose.position.x = testv[0];
                p.pose.position.y = testv[1];
                p.pose.position.z = waypoints.waypoints[index].z_alt;
                pose.push_back(p);
                ROS_INFO("pose.size() = %d, command = %d, p.pose.position.x = %f, p.pose.position.y = %f, "
                         "p.pose.position.z = %f",
                         static_cast<int>(pose.size()), waypoints.waypoints[index].command, p.pose.position.x,
                         p.pose.position.y, p.pose.position.z);
                //*****************************************************************************************
            }
        }

        if (waypoints.waypoints.size() == 0)
        {
            ROS_WARN_THROTTLE(1, "No waypoints received yet. Waiting for waypoints...");
        }
        else
        {
            break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = init_position_x_take_off + 0;
    setpoint_raw.position.y = init_position_y_take_off + 0;
    setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;

    for (int i = 100; ros::ok() && i > 0; --i)
    {
        setpoint_raw_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();
    bool offboard_flag = false;
    // lib_pwm_control(0, 0);
    // ctrl_pwm_client.call(lib_ctrl_pwm);

    while (ros::ok())
    {
        if (offboard_flag == false && current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
                offboard_flag = true;
            }

            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }

                last_request = ros::Time::now();
            }
        }

        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.5)
        {
            if (ros::Time::now() - last_request > ros::Duration(3.0))
            {
                break;
            }
        }

        setpoint_raw_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    //开始录制视频
    // startRosbagRecord();

    for (int i = 0; i < pose.size();)
    {
        while (ros::ok())
        {
            //判断是否看到目标
            if (break_to_recognize_permission == true)
            {
                cout << "发现目标，定位中" << endl;

                if (object_recognize_track_vel_yolo8("landing", 0, ALTITUDE, 0.5, 80))
                {
                    if (lib_time_record_func(0.2, ros::Time::now()))
                    {
                        target_exact_coordinates_permission = true;
                        break_to_recognize_permission = false;
                        break;
                    }
                }
                else
                {
                    if (lib_time_record_func(120.0, ros::Time::now()))
                    {
                        target_exact_coordinates_permission = true;
                        break_to_recognize_permission = false;
                        break;
                    }
                }
            }
            else
            {
                cout << "第	" << i << "	个航点巡航中" << endl;

                if (fabs(local_pos.pose.pose.position.x - pose[i].pose.position.x) < 1.0 &&
                    fabs(local_pos.pose.pose.position.y - pose[i].pose.position.y) < 1.0)
                {
                    i++;
                    cout << "到达第" << i << "个巡航点" << endl;
                    break; //当前位置与期望航点距离相差不到1m,则退出此次航点任务
                }
                else
                {
                    setpoint_raw.coordinate_frame = 1;
                    setpoint_raw.type_mask = /*1 + 2 + 4 + */ 8 + 16 + 32 + 64 + 128 + 256 + 512 + 1024 + 2048;
                    setpoint_raw.position.x = pose[i].pose.position.x;
                    setpoint_raw.position.y = pose[i].pose.position.y;
                    setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
                }
            }
            gps_udp_msg.header.frame_id  = obj_pos_.name;
            gps_udp_msg.latitude = glo_pos_msg.latitude;
            gps_udp_msg.longitude = glo_pos_msg.longitude;
            if(gps_udp_msg.latitude != 0){
            	gps_udp_pub.publish(gps_udp_msg);
            }

            setpoint_raw_pub.publish(setpoint_raw);
            ros::spinOnce();
            rate.sleep();
        }

        //跑完所有航点后则跳出，准备进入其他的程序
        if ((i + 1) > pose.size())
        {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "AUTO.RTL";

            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("RTL enabled");
                ROS_INFO("mission end,shutdown this node!!!");
                break;
            }
        }
    }

    // 节点关闭时停止rosbag
    // stopRosbagRecord();
    ros::shutdown();
    return 0;
}
