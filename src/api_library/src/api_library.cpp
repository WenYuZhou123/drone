#include <api_library.h>

//1、无人机状态回调函数
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//2、回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);	
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    if (flag_init_position==false && (local_pos.pose.pose.position.z!=0))
    {
		init_position_x_take_off = local_pos.pose.pose.position.x;
	    init_position_y_take_off = local_pos.pose.pose.position.y;
	    init_position_z_take_off = local_pos.pose.pose.position.z;
	    init_yaw_take_off        = yaw;
        flag_init_position = true;		    
    } 
}

//3、室内外通用自主巡航函数API
bool mission_pos_cruise(float x, float y, float z, float yaw, float error_max)
{
	if(error_max<=0)
	{
		error_max = 0.5;
		ROS_INFO("室外巡航，误差允许值不被允许，GPS允许误差需要大于0");
		ROS_INFO("已经重置为默认参数, error_max = 0.5");
	}

	
	if(mission_pos_cruise_flag == false)
	{
		mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
		mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
		mission_pos_cruise_flag = true;
		ROS_INFO("发布巡航目标点 x = %f, y = %f, z = %f, yaw = %f",x, y , z, yaw);
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = init_position_x_take_off + x;
	setpoint_raw.position.y = init_position_y_take_off + y;
	setpoint_raw.position.z = init_position_z_take_off + z;
	setpoint_raw.yaw        = yaw;             
	if(fabs(local_pos.pose.pose.position.x-x-init_position_x_take_off)<error_max && fabs(local_pos.pose.pose.position.y-y-init_position_y_take_off)<error_max)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		mission_pos_cruise_flag = false;
		return true;
	}
	return false;
}


//4、室内外目标识别，采用速度控制运动，任务是识别到目标后，保持无人机正对着目标
bool object_recognize_track_vel(string str, float yaw, float ALTITUDE, float speed, float error_max)
{
	if(error_max<=0)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else if(error_max>=240)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else
	{
	}
	//此处false主要是为了获取任务初始时候的位置信息
	if(object_recognize_track_vel_flag == false)
	{
		//获取初始位置，防止无人机飘
		object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_vel_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == str)
    {
		//此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_vel_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_vel_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 
        ROS_INFO("position_detec_x  = %f\r",    position_detec_x);
        ROS_INFO("position_detec_y  = %f\r",    position_detec_y);
        ROS_INFO("position_detec_z  = %f\r\n",  position_detec_z);

		if(fabs(position_detec_x-320) < error_max && fabs(position_detec_y-240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
		else
		{
			//摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
			//无人机左右移动速度控制,也可以采用位置控制。
			//速度控制响应更快，建议室外采用速度控制，室内采用位置控制
			if(position_detec_x -320 >= error_max)
			{
				setpoint_raw.velocity.y =  -speed;
			}		
			else if(position_detec_x - 320 <= -error_max)
			{
				setpoint_raw.velocity.y =  speed;
			}	
			else
			{
				setpoint_raw.velocity.y =  0;
			}					  
			//无人机前后移动速度控制
			if(position_detec_y -240 >= error_max)
			{
				setpoint_raw.velocity.x =  -speed;
			}
			else if(position_detec_y - 240 <= -error_max)
			{
				setpoint_raw.velocity.x =  speed;
			}
			else
			{
				setpoint_raw.velocity.x =  0;
			}
			//机体坐标系下发送xy速度期望值以及高度z期望值至飞控（输入：期望xy,期望高度）
        	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 + 1024 + 2048;
			setpoint_raw.coordinate_frame = 8;
		    setpoint_raw.position.z = init_position_z_take_off+ALTITUDE;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_last_time_position_y;
		setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
		setpoint_raw.coordinate_frame = 1;
		setpoint_raw.position.z = init_position_z_take_off + ALTITUDE;
		setpoint_raw.yaw        = yaw;
	}
	return false;
}


//5、无人机追踪函数，仅仅跟踪水平方向，即Y方向，通常用于无人机穿越圆框、方框等使用
bool object_recognize_track(string str, float yaw, int reverse, float ALTITUDE, float error_max,float ctrl_coef)
{
	if(error_max<=0)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else if(error_max>=240)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else
	{
	}
	
	if(object_recognize_track_flag == false)
	{
		object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == str)
    {
		//此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 
        /*ROS_INFO("position_detec_x  = %f\r",    position_detec_x);
        ROS_INFO("position_detec_y  = %f\r",    position_detec_y);
        ROS_INFO("position_detec_z  = %f\r\n",  position_detec_z);
        */
		//摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if(position_detec_x -320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef*reverse;
		}					
		else if(position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}
	
		if(fabs(position_detec_x-320) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = 0 + ALTITUDE;
	setpoint_raw.yaw        = yaw;
	return false;
}

//6、无人机跟踪函数，包含X，Y两个方向，相当于全向跟踪
bool object_recognize_track_omni(string str, float yaw, int reverse, float ALTITUDE,float error_max, float ctrl_coef)
{
	if(error_max<=0)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else if(error_max>=240)
	{
		error_max = 20;
		ROS_INFO("误差允许值不被允许，应该设置在0-240之间");
		ROS_INFO("已经重置为默认参数, error_max = 20");
	}
	else
	{
	}

	//此处false主要是为了获取任务初始时候的位置信息
	if(object_recognize_track_omni_flag == false)
	{
		//获取初始位置，防止无人机飘
		object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
    	object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
		object_recognize_track_omni_flag = true;
		ROS_INFO("开始识别目标并且保持跟踪");
	}
	//此处首先判断是否识别到指定的目标
	if(object_pos.header.frame_id == str)
    {
		//此处实时更新当前位置，防止目标丢失的时候导致无人机漂移
        object_recognize_track_omni_last_time_position_x = local_pos.pose.pose.position.x;
        object_recognize_track_omni_last_time_position_y = local_pos.pose.pose.position.y;
        //获取到目标物体相对摄像头的坐标
        position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 
        /*ROS_INFO("position_detec_x  = %f\r",    position_detec_x);
        ROS_INFO("position_detec_y  = %f\r",    position_detec_y);
        ROS_INFO("position_detec_z  = %f\r\n",  position_detec_z);
        */
		//摄像头向下或者向前安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
		if(position_detec_x -320 >= error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef*reverse;
		}					
		else if(position_detec_x - 320 <= -error_max)
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.y = local_pos.pose.pose.position.y;
		}
		

		if(position_detec_y -240 >= error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x - ctrl_coef*reverse;
		}					
		else if(position_detec_y - 240 <= -error_max)
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x + ctrl_coef*reverse;
		}
		else
		{
			setpoint_raw.position.x = local_pos.pose.pose.position.x;
		}	

		if(fabs(position_detec_x-320) < error_max && fabs(position_detec_y-240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方，在允许误差范围内保持");
			return true;
		}
	}
	else
	{	
		setpoint_raw.position.x = object_recognize_track_omni_last_time_position_x;
		setpoint_raw.position.y = object_recognize_track_omni_last_time_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = 0 + ALTITUDE;
	setpoint_raw.yaw        = yaw;
	return false;
}




bool  object_recognition_land(string str,float land_threshold,float yaw, float ALTITUDE, float error_max, float ctrl_coef)
{
	//此处false主要是为了获取任务初始时候的位置信息
	if(object_recognition_land_init_position_flag == false)
	{
		//获取初始位置，防止无人机飘
		object_recognition_land_last_position_x = local_pos.pose.pose.position.x;
    	object_recognition_land_last_position_y = local_pos.pose.pose.position.y;
		object_recognition_land_init_position_flag = true;
		ROS_INFO("开始目标识别并降落");
	}
	if(object_pos.header.frame_id == str)
    {
		object_recognition_land_last_position_x = local_pos.pose.pose.position.x;
		object_recognition_land_last_position_y = local_pos.pose.pose.position.y;
    	position_detec_x = object_pos.point.x;
        position_detec_y = object_pos.point.y;
        position_detec_z = object_pos.point.z; 
        
        //ROS_INFO("position_detec_x  = %f\r",    position_detec_x);
        //ROS_INFO("position_detec_y  = %f\r",    position_detec_y);
        //ROS_INFO("position_detec_z  = %f\r\n",  position_detec_z);
		if(fabs(position_detec_x-320) < error_max && fabs(position_detec_y-240) < error_max)
		{
			ROS_INFO("到达目标的正上/前方");	
			return true;		
		}
		else
		{
			if(position_detec_x -320 >= error_max)
			{
				setpoint_raw.position.y = local_pos.pose.pose.position.y - ctrl_coef;
			}					
			else if(position_detec_x - 320 <= -error_max)
			{
				setpoint_raw.position.y = local_pos.pose.pose.position.y + ctrl_coef;
			}
			else
			{
				setpoint_raw.position.y = local_pos.pose.pose.position.y;
			}
			if(position_detec_y -240 >= error_max)
			{
				setpoint_raw.position.x = local_pos.pose.pose.position.x - ctrl_coef;
			}					
			else if(position_detec_y - 240 <= -error_max)
			{
				setpoint_raw.position.x = local_pos.pose.pose.position.x + ctrl_coef;
			}
			else
			{
				setpoint_raw.position.x = local_pos.pose.pose.position.x;
			}	
		}
	}
	else
	{
		setpoint_raw.position.x = object_recognition_land_last_position_x;
		setpoint_raw.position.y = object_recognition_land_last_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = ALTITUDE;
	setpoint_raw.yaw        = 0;
	return false;
}

//获取识别目标的位置信息
void object_pos_cb(const geometry_msgs::PointStamped::ConstPtr& msg)
{
	object_pos = *msg;
}





void ar_marker_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
	int count = msg->markers.size();
	if(count!=0)
	{
		marker_found = true;
		for(int i = 0; i<count; i++)
		{
			marker = msg->markers[i];
			ar_track_id_current = marker.id;			
		}
	
	}
	else
	{
		marker_found = false;
	}
}


bool ar_lable_land(float marker_error_max, int ar_track_id, float ALTITUDE)
{
	if(ar_lable_land_init_position_flag == false)
	{
		//获取初始位置，防止无人机飘
		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
    	ar_lable_land_last_position_y = local_pos.pose.pose.position.y;
		ar_lable_land_init_position_flag = true;
		ROS_INFO("进入二维码识别和降落");
	}
	//识别到指定的id才进入这个循环，否则不控制
	if(marker.id == ar_track_id)
    {
    	//此处根本摄像头安装方向，进行静态坐标转换
        marker_x = marker.pose.pose.position.x;
        marker_y = marker.pose.pose.position.y;
        marker_z = marker.pose.pose.position.z; 
        printf("marker_x = %f\r\n",marker_x);
        printf("marker_y = %f\r\n",marker_y);
        printf("marker_z = %f\r\n",marker_z);
    	printf("11\r\n");
		ar_lable_land_last_position_x = local_pos.pose.pose.position.x;
		ar_lable_land_last_position_y = local_pos.pose.pose.position.y;	
		if(fabs(marker_x) < marker_error_max && fabs(marker_y) < marker_error_max)
		{
			ROS_INFO("到达目标的正上/前方");	
			return true;
		}
		else
		{	
			printf("22\r\n");
			//摄像头朝下安装，因此摄像头的Z对应无人机的X前后方向，Y对应Y左右方向，Z对应上下
			//无人机左右移动速度控制
			if(marker_x >= marker_error_max)
			{
				printf("31\r\n");
				setpoint_raw.position.y = local_pos.pose.pose.position.y - 0.1;
			}					
			else if(marker_x <= -marker_error_max)
			{
				printf("32\r\n");
				setpoint_raw.position.y = local_pos.pose.pose.position.y + 0.1;
			}	
			else
			{
				printf("33\r\n");
				setpoint_raw.position.y = ar_lable_land_last_position_y;
			}					  
			//无人机前后移动速度控制
			if(marker_y >= marker_error_max)
			{
				printf("34\r\n");
				setpoint_raw.position.x = local_pos.pose.pose.position.x - 0.1;
			}
			else if(marker_y <= -marker_error_max)
			{
				printf("35\r\n");
				setpoint_raw.position.x = local_pos.pose.pose.position.x + 0.1;
			}
			else
			{
				printf("36\r\n");
				setpoint_raw.position.x = ar_lable_land_last_position_x;
			}
		}				
	}
	else
	{	
		printf("44\r\n");
		setpoint_raw.position.x = ar_lable_land_last_position_x;
		setpoint_raw.position.y = ar_lable_land_last_position_y;
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = ALTITUDE;
	setpoint_raw.yaw        = 0;
	return false;
}


bool current_position_cruise(float x, float y, float z, float yaw)
{
	if(current_position_cruise_flag == false)
	{
		current_position_cruise_last_position_x = local_pos.pose.pose.position.x;
		current_position_cruise_last_position_y = local_pos.pose.pose.position.y;
		ROS_INFO("current_position_cruise_last_position_x = %f",current_position_cruise_last_position_x);
		ROS_INFO("current_position_cruise_last_position_y = %f",current_position_cruise_last_position_y);
		current_position_cruise_flag = true;
		ROS_INFO("发布巡航目标点 x = %f, y = %f, z = %f, yaw = %f",x, y , z, yaw);
	}
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.x = current_position_cruise_last_position_x + x;
	setpoint_raw.position.y = current_position_cruise_last_position_y + y;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw        = yaw;
	ROS_INFO("local_pos.pose.pose.position.x = %f",local_pos.pose.pose.position.x); 
	ROS_INFO("local_pos.pose.pose.position.y = %f",local_pos.pose.pose.position.y);                         
	if(fabs(local_pos.pose.pose.position.x-current_position_cruise_last_position_x-x)<0.1 && fabs(local_pos.pose.pose.position.y-current_position_cruise_last_position_y-y)<0.1)
	{
		ROS_INFO("到达目标点，巡航点任务完成");
		current_position_cruise_flag = false;
		return true;
	}
	return false;
}


void CmdVelCallback(const geometry_msgs::Twist &msg)
{
	setpoint_raw.type_mask = 1 + 2 +/* 4 + 8 + 16 + 32 +*/ 64 + 128 + 256 + 512 + 1024/* + 2048*/;
	setpoint_raw.coordinate_frame = 8;
	setpoint_raw.velocity.x = msg.linear.x;;
	setpoint_raw.velocity.y = msg.linear.y;
	setpoint_raw.position.z = 0.8;
	setpoint_raw.yaw_rate   = msg.angular.z;
} 

bool target_through(float pos_x, float pos_y, float z, float yaw)
{
	//初始化位置一次即可，用于获取无人机初始位置
	if(!target_through_init_position_flag)
	{
		target_through_init_position_x = local_pos.pose.pose.position.x;
		target_through_init_position_y = local_pos.pose.pose.position.y;
		target_through_init_position_flag = true;
	}
	setpoint_raw.position.x = target_through_init_position_x + pos_x;
	setpoint_raw.position.y = target_through_init_position_y + pos_y;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
	setpoint_raw.position.z = z;
	setpoint_raw.yaw        = yaw;
	if(fabs(local_pos.pose.pose.position.x-pos_x-target_through_init_position_x)<0.1 && fabs(local_pos.pose.pose.position.y-pos_y-target_through_init_position_y)<0.1)
	{
		ROS_INFO("到达目标点，穿越圆框任务完成");
		return true;
	}
	return false;	
}



void precision_land()
{
	//初始化位置一次即可，用于获取无人机初始位置
	if(!precision_land_init_position_flag)
	{
		precision_land_init_position_x = local_pos.pose.pose.position.x;
		precision_land_init_position_y = local_pos.pose.pose.position.y;
		precision_land_init_position_flag = true;
	}
	setpoint_raw.position.x = precision_land_init_position_x;
	setpoint_raw.position.y = precision_land_init_position_y;
	setpoint_raw.position.z = -0.15;
	setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
	setpoint_raw.coordinate_frame = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "api_library");
    
    ros::NodeHandle nh;
    
    ros::Rate rate(10.0);
    
    return 0;
}


