/****************************************************************

*****************************************************************/
#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <mavros_msgs/RCIn.h>
#include <tf/transform_listener.h>
#include <bitset>

/**************************************
自检变量不同位表示不同含义，暂定义如下1为真，0为假：

位1：表示电池电压正常
位2：表示遥控器是否连接
位3：表示mavros通信已经连接
位4：表示里程计初始数据正常

其他：待定
***************************************/
unsigned int check_result = 0;

void current_battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{
	if (msg->voltage >= 15.2)
		check_result |= 0b00000001;
	else
		check_result &= ~0b00000001;
}

mavros_msgs::RCIn global_rc_in;
void rc_in_cb(const mavros_msgs::RCIn::ConstPtr& msg)
{
	global_rc_in = *msg;
	if (global_rc_in.channels[4] >= 1200 && global_rc_in.channels[4] <= 1500)
		check_result &= ~0b00000010;
	else
		check_result |= 0b00000010;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
	current_state = *msg;
	if (msg->connected)
		check_result |= 0b00000100;
	else
		check_result &= ~0b00000100;
}

nav_msgs::Odometry local_pos;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
bool flag_init_position = false;
tf::Quaternion quat;
double roll, pitch, yaw;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
	local_pos = *msg;
	if (!flag_init_position && local_pos.pose.pose.position.z != 0)
	{
		init_position_x_take_off = local_pos.pose.pose.position.x;
		init_position_y_take_off = local_pos.pose.pose.position.y;
		init_position_z_take_off = local_pos.pose.pose.position.z;

		if (fabs(init_position_x_take_off) < 0.15 && fabs(init_position_y_take_off) < 0.15 && fabs(init_position_z_take_off) < 0.15)
		{
			check_result |= 0b00001000;
			flag_init_position = true;
		}
	}
	tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "self_check");
	ros::NodeHandle nh;

	ros::Subscriber state_sub = nh.subscribe("mavros/state", 10, state_cb);
	ros::Subscriber local_pos_sub = nh.subscribe("/mavros/local_position/odom", 10, local_pos_cb);
	ros::Subscriber current_battery_sub = nh.subscribe("/mavros/battery", 10, current_battery_cb);
	ros::Subscriber rc_in_sub = nh.subscribe("/mavros/rc/in", 100, rc_in_cb);

	ros::Rate rate(20.0);

	while (ros::ok())
	{
		std::cout << std::bitset<sizeof(check_result) * 8>(check_result) << std::endl;
		std::cout << check_result << std::endl;
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
