//说明：舵机方向根据实际调整，这里用的占空比0%表示闭合，100%表示打开
#include <api_library.h>
#include <lib_library.h>
#define ALTITUDE  0.8


int mission_num = 0; 

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
bool mission_obs_avoid_flag = false;
bool mission_obs_avoid(float x, float y, float z, float yaw)
{
	if(mission_obs_avoid_flag == false)
	{
		mission_obs_avoid_flag = true;
		ROS_INFO("发布避障导航目标点 x = %f, y = %f, z = %f, yaw = %f",x, y , z, yaw);
	}
	MoveBaseClient ac("move_base", true);
	while(!ac.waitForServer(ros::Duration(1.0)))
	{
	    ROS_INFO("Waiting for the move_base action server to come up");
	}
	move_base_msgs::MoveBaseGoal first_goal;
	first_goal.target_pose.header.frame_id = "map";
	first_goal.target_pose.header.stamp = ros::Time::now();
	first_goal.target_pose.pose.position.x = x;
	first_goal.target_pose.pose.position.y = y;
	first_goal.target_pose.pose.position.z = ALTITUDE;
	first_goal.target_pose.pose.orientation.w = 1;
    ac.sendGoal(first_goal);
    if(fabs(local_pos.pose.pose.position.x-x)<0.2 && fabs(local_pos.pose.pose.position.y-y)<0.2)
	{
		ROS_INFO("到达目标点，自主导航任务完成");
		return true;
	}
	return false;
}


void cmd_to_px4(const geometry_msgs::Twist &msg)
{
	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	setpoint_raw.coordinate_frame = 8;
	setpoint_raw.velocity.x = msg.linear.x;;
	setpoint_raw.velocity.y = msg.linear.y;
	setpoint_raw.position.z = ALTITUDE;
	setpoint_raw.yaw_rate   = msg.angular.z;
} 


int main(int argc, char** argv)
{
	setlocale(LC_ALL, "");

	ros::init(argc, argv, "complete_mission");

  	ros::NodeHandle nh; 

	//状态回调
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);   

	//里程计订阅
	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

	//订阅图片位置
    ros::Subscriber object_pos_sub = nh.subscribe<geometry_msgs::PointStamped>("object_position", 100, object_pos_cb); 

	//订阅move_base发布的cmd_vel速度信息
    ros::Subscriber cmd_vel_sub    = nh.subscribe("/cmd_vel", 10,cmd_to_px4);

	//订阅二维码位置和姿态信息
    ros::Subscriber ar_pos_sub = nh.subscribe("/ar_pose_marker", 100, ar_marker_cb);

	//发布无人机控制数据
	ros::Publisher  mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);  
	 
	//创建无人机申请解锁客户端
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    
    //创建无人机模式切换客户端
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");   

	//创建舵机控制客户端
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

	//设置节点运行频率
    ros::Rate rate(20);  
    	
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    //此处控制无人机的位置、速度、偏航角和角速度，同时控制的话，优先上一级，如位置和速度，优先位置控制，速度不起效果
    //飞行到指定的高度，添加了位置漂移误差
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = init_position_x_take_off;
	setpoint_raw.position.y = init_position_y_take_off;
	setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
	setpoint_raw.yaw= 0;

    for(int i = 100; ros::ok() && i > 0; --i)
    {
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
	//申请切入offboard机载控制模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    //请求解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

	//记录当前时间辍
    ros::Time last_request = ros::Time::now();

	//控制飞控5和6号口舵机按照0%的占空比
    lib_pwm_control(0,0);
    ctrl_pwm_client.call(lib_ctrl_pwm);
    
    //循环请求进入offboard和解锁armed
    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
           	flag_init_position = false;		    
       	}
        else 
		{
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        last_request = ros::Time::now();
		        flag_init_position = false;		    
			}
		}
	    
	    //1、添加高度判断，使得无人机跳出模式切换循环
	    if(fabs(local_pos.pose.pose.position.z- init_position_z_take_off -ALTITUDE)<0.1)
		{	
			if(ros::Time::now() - last_request > ros::Duration(3.0))
			{
				mission_num = 1;
				break;
			}
		}
		//2、添加时间判断，使得无人机跳出模式切换循环
		if(ros::Time::now() - last_request > ros::Duration(10.0))
		{
			mission_num = 1;
			break;
		}
		//此处添加是为增加无人机的安全性能，在实际测试过程中，采用某款国产的GPS和飞控，气压计和GPS定位误差极大，
		//导致了无人机起飞后直接飘走，高度和位置都不正常，无法跳出模式循环，导致遥控且无法接管
		//因此增加了时间判断，确保无人机在切入offboard模式和解锁后，确保任何情况下，8秒后遥控器都能切入其他模式接管无人机	
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		//注意：一定要确定GPS和飞控传感器都是正常的
		mission_pos_cruise(0, 0, ALTITUDE, 0, 0.2);		
		mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
    //此处为任务循环函数，顺序执行即可 
    float tmp_x = 0;
    float tmp_y = 0;
	while(ros::ok())
    {
    	printf("mission_num = %d\r\n",mission_num);
    	switch(mission_num)
    	{	
    		case 1:
    		    if(mission_pos_cruise(0, 0, ALTITUDE, 0, 0.3))
    			{
    				if(lib_time_record_func(2.0,ros::Time::now()))
    				{
    					mission_num=2;
    				}
    			}
    			break; 

    	    case 2:
    		    if(mission_obs_avoid(3.0, 0, ALTITUDE, 0))
    			{
    				if(lib_time_record_func(1.0,ros::Time::now()))
    				{
    					mission_num=21;
    				}
    			}
    			break;
    		case 21:
    		    if(mission_pos_cruise(3, 0, ALTITUDE, 0, 0.2))
    			{
    				if(lib_time_record_func(1.0,ros::Time::now()))
    				{
    					mission_num=22;
    				}
    			}
    			break;
    		
    		case 22:
				if(object_pos.header.frame_id == "armoredcar" || object_pos.header.frame_id == "bridge")
    			{
    				if(object_recognize_track_vel(object_pos.header.frame_id, 0, ALTITUDE, 0.10, 30))
					{
						if(lib_time_record_func(1.0,ros::Time::now()))
						{
							tmp_x = local_pos.pose.pose.position.x;
							tmp_y = local_pos.pose.pose.position.y;
							//lib_pwm_control(100,100);
							//ctrl_pwm_client.call(lib_ctrl_pwm);
							//ROS_INFO("舵机执行物体投放");
							mission_num=23;
						}
					}
    			}
    			else
    			{
    				if(lib_time_record_func(10.0,ros::Time::now()))
					{
						mission_num=3;
					}
    			}
    			break;	
    		
    		case 23:
    			printf("tmp_x = %f\r\n",tmp_x);
    			printf("tmp_y = %f\r\n",tmp_y);    			
    		    if(mission_pos_cruise(tmp_x, tmp_y, 0.2, 0, 0.2))
    			{
    				if(lib_time_record_func(5.0,ros::Time::now()))
    				{
    					lib_pwm_control(100,100);
						ctrl_pwm_client.call(lib_ctrl_pwm);
						ROS_INFO("舵机执行物体投放");
    					mission_num=3;
    				}
    			}
    			break;	
    			
    		case 3:
    		    if(mission_obs_avoid(3, 3, ALTITUDE, 0))
    			{
    				if(lib_time_record_func(1.0,ros::Time::now()))
    				{
    					mission_num=31;
    				}
    			}
    			break; 
    			
    			
    		case 31:
    		    if(mission_pos_cruise(3, 3, ALTITUDE, 0, 0.2))
    			{
    				if(lib_time_record_func(1.0,ros::Time::now()))
    				{
    					mission_num=32;
    				}
    			}
    			break;
    		
    		case 32:
				if(object_pos.header.frame_id == "armoredcar" || object_pos.header.frame_id == "bridge")
    			{
    				if(object_recognize_track_vel(object_pos.header.frame_id, 0, ALTITUDE, 0.10, 30))
					{
						if(lib_time_record_func(0.1,ros::Time::now()))
						{
							tmp_x = local_pos.pose.pose.position.x;
							tmp_y = local_pos.pose.pose.position.y;
							mission_num=33;
						}
					}
    			}
    			else
    			{
    				if(lib_time_record_func(10.0,ros::Time::now()))
					{
						mission_num=4;
					}
    			}
    			break;	
    		case 33:
    			printf("tmp_x = %f\r\n",tmp_x);
    			printf("tmp_y = %f\r\n",tmp_y);    			
    		    if(mission_pos_cruise(tmp_x, tmp_y, 0.2, 0, 0.2))
    			{
    				if(lib_time_record_func(5.0,ros::Time::now()))
    				{
    					lib_pwm_control(100,100);
						ctrl_pwm_client.call(lib_ctrl_pwm);
						ROS_INFO("舵机执行物体投放");
    					mission_num=4;
    				}
    			}
    			break;	
    			
    		case 4:
    		    if(mission_pos_cruise(0, 2, ALTITUDE, 0, 0.2))
    			{
    				if(lib_time_record_func(1.0,ros::Time::now()))
    				{
    					mission_num=42;
    				}
    			}
    			break;
    		
    		case 42:
				if(object_pos.header.frame_id == "apron" || object_pos.header.frame_id == "apron")
    			{
    				if(object_recognize_track_vel(object_pos.header.frame_id, 0, ALTITUDE, 0.10, 30))
					{
						if(lib_time_record_func(0.1,ros::Time::now()))
						{
							tmp_x = local_pos.pose.pose.position.x;
							tmp_y = local_pos.pose.pose.position.y;
							mission_num=43;
						}
					}
    			}
    			else
    			{
    				if(lib_time_record_func(10.0,ros::Time::now()))
					{
						mission_num=44;
					}
    			}
    			break;	
    		case 43:
    			printf("tmp_x = %f\r\n",tmp_x);
    			printf("tmp_y = %f\r\n",tmp_y);    			
    		    if(mission_pos_cruise(tmp_x, tmp_y, 0.2, 0, 0.2))
    			{
    				if(lib_time_record_func(5.0,ros::Time::now()))
    				{ 
    					mission_num=44;
    				}
    			}
    			break;	   			
    		case 44:
    		    //if(mission_pos_cruise(0, 2, ALTITUDE, 0, 0.2))
    			//{
    				//if(lib_time_record_func(1.0,ros::Time::now()))
    				//{
    					ROS_INFO("AUTO.LAND");
						offb_set_mode.request.custom_mode = "AUTO.LAND";
						set_mode_client.call(offb_set_mode);
    					mission_num=44;
    				//}
    			//}
    			break;
    			
			
    	}
    	mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }
	return 0;
}










