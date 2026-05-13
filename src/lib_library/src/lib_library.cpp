#include <lib_library.h>


//1、无人机状态回调函数
void lib_state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    lib_current_state = *msg;
}


//2、回调函数接收无人机的里程计信息
void lib_local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    lib_local_pos = *msg;
    if (lib_flag_init_position==false && (lib_local_pos.pose.pose.position.z!=0))
    {
		lib_init_position_x_take_off = lib_local_pos.pose.pose.position.x;
	    lib_init_position_y_take_off = lib_local_pos.pose.pose.position.y;
	    lib_init_position_z_take_off = lib_local_pos.pose.pose.position.z;
        lib_flag_init_position = true;		    
    }
    tf::quaternionMsgToTF(lib_local_pos.pose.pose.orientation, lib_quat);	
	tf::Matrix3x3(lib_quat).getRPY(lib_roll, lib_pitch, lib_yaw);
}


//3、延时函数，每个任务完成后，给予time_duration时间进行延时，方便进行下一个操作，否则无视觉停留效果    
bool lib_time_record_func(float time_duration,ros::Time time_now)
{
	if(lib_time_record_start_flag == false)
	{
		lib_mission_success_time_record = time_now;
		lib_time_record_start_flag = true;
	}
	if(ros::Time::now() -lib_mission_success_time_record > ros::Duration(time_duration))
	{
		lib_time_record_start_flag = false;
		return true;
	}
	else
	{
		return false;
	}
}

//4、PWM控制舵机函数
void lib_pwm_control(int pwm_channel_5, int pwm_channel_6)
{
     lib_ctrl_pwm.request.command = 187;    
     lib_ctrl_pwm.request.param1 = ((pwm_channel_5/50.0)-1);        
     lib_ctrl_pwm.request.param2 = ((pwm_channel_6/50.0)-1);     
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lib_library");
    
    ros::NodeHandle nh;
    
    ros::Rate rate(10.0);
    
    return 0;
}


