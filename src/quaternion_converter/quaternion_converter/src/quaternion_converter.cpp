#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Quaternion.h>
#include <mavros_msgs/ADSBVehicle.h>
#include <cmath>

class QuaternionConverter
{
public:
    // 将欧拉角转换为四元数
    geometry_msgs::Quaternion eulerToQuaternion(double roll, double pitch, double yaw)
    {
        // 创建四元数对象
        tf2::Quaternion quaternion;

        // 设置欧拉角（单位为弧度）
        quaternion.setRPY(roll, pitch, yaw);

        // 将四元数转换为消息格式
        geometry_msgs::Quaternion quat_msg;
        quat_msg.x = quaternion.x();
        quat_msg.y = quaternion.y();
        quat_msg.z = quaternion.z();
        quat_msg.w = quaternion.w();

        return quat_msg;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion_converter_node");
    ros::NodeHandle nh;
    
    // 发布 ADSB 信息
    ros::Publisher adsb_pub = nh.advertise<mavros_msgs::ADSBVehicle>("/mavros/adsb/send", 10);

    // 控制循环频率
    ros::Rate rate(20.0);

    // 创建四元数转换器对象
    QuaternionConverter converter;

    // 初始化欧拉角
    double roll = 0, pitch = M_PI/2, yaw = M_PI;

    // 循环读取参数
    while (ros::ok())
    {
        // 从参数服务器获取 roll、pitch、yaw 的值
        nh.getParam("q_roll", roll);
        nh.getParam("q_pitch", pitch);  // 如果需要获取 pitch 的值
        nh.getParam("q_yaw", yaw);      // 如果需要获取 yaw 的值

        // 转换为四元数
        geometry_msgs::Quaternion quat = converter.eulerToQuaternion(roll, pitch, yaw);

        // 填充 ADSB 信息
        mavros_msgs::ADSBVehicle adsb_msg;
        adsb_msg.latitude     = quat.x;   // 请确认是否应该使用 quat.x 作为纬度
        adsb_msg.longitude    = quat.y;   // 同上，确认是否应使用 quat.y 作为经度
        adsb_msg.altitude     = quat.z;
        adsb_msg.hor_velocity = quat.w;

        // 输出四元数信息
        ROS_INFO("Quat: [x: %f, y: %f, z: %f, w: %f]", quat.x, quat.y, quat.z, quat.w);

        // 发布 ADSB 信息
        adsb_pub.publish(adsb_msg);

        // 处理ROS事件
        ros::spinOnce();

        // 控制循环频率
        rate.sleep();
    }

    return 0;
}

