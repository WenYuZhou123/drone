#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define ALTITUDE 0.5

//定义变量，用于接收无人机的状态信息
mavros_msgs::State current_state;
//定义变量，用于接收无人机的里程计信息
nav_msgs::Odometry local_pos;

//回调函数接收无人机的状态信息
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//回调函数接收无人机的里程计信息
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
}
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_navigation_goals");
  
  //创建节点句柄
  	ros::NodeHandle nh;
    
	ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

	//创建一个Subscriber订阅者，订阅名为/mavros/state的topic，注册回调函数state_cb
	ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
	   
	//创建一个Subscriber订阅者，订阅名为/mavros/local_position/odom的topic，注册回调函数local_pos_cb
	ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

	//创建一个服务客户端，连接名为/mavros/cmd/arming的服务，用于请求无人机解锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

    //创建一个服务客户端，连接名为/mavros/set_mode的服务，用于请求无人机进入offboard模式
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //设置话题发布频率，需要大于2Hz，飞控连接有500ms的心跳包
    ros::Rate rate(200.0);

    //等待连接到飞控
    while(ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    //设置无人机的期望位置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x =   0;
    pose.pose.position.y =   0;
    pose.pose.position.z =   ALTITUDE;  

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    //定义客户端变量，设置为offboard模式
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    
    //定义客户端变量，请求无人机解锁
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    //记录当前时间，并赋值给变量last_request
    ros::Time last_request = ros::Time::now();
 
 	//此处满足一次请求进入offboard模式即可，官方例成循环切入offboard会导致无人机无法使用遥控器控制
    while(ros::ok())
    {
    	//请求进入OFFBOARD模式
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else 
		{
			//请求解锁
			if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
		        if( arming_client.call(arm_cmd) && arm_cmd.response.success)
		       	{
		            ROS_INFO("Vehicle armed");
		        }
		        	last_request = ros::Time::now();
			}
		}
		
		//当无人机到达起飞点高度后，悬停3秒后进入任务模式，提高视觉效果
		if(fabs(local_pos.pose.pose.position.z-ALTITUDE)<0.1)
		{
			if(ros::Time::now() - last_request > ros::Duration(2.0))
			{
				break;
			}
		}

	    //发布期望位置信息
		local_pos_pub.publish(pose);
		
		// 循环等待回调函数
        ros::spinOnce();
        
        // 按照循环频率延时
        rate.sleep();
    }   
  
 
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
 
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(1.0)))
{
    ROS_INFO("Waiting for the move_base action server to come up");
}
 
  move_base_msgs::MoveBaseGoal first_goal;
  move_base_msgs::MoveBaseGoal second_goal;


  // set up the frame parameters
  first_goal.target_pose.header.frame_id = "map";
  first_goal.target_pose.header.stamp = ros::Time::now();
  

  // Define a position and orientation for the robot to reach
  first_goal.target_pose.pose.position.x = 3.0;
  first_goal.target_pose.pose.position.y = 0.0;
  first_goal.target_pose.pose.position.z = ALTITUDE;
  first_goal.target_pose.pose.orientation.w = 1;



 
  // Send the goal 1
  ROS_INFO("Sending first_goal");
  ac.sendGoal(first_goal);
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) 
  {
    ROS_INFO("Report, the base moved to the first goal");
  }
  else 
  {
    ROS_INFO("The base failed to move to the first goal");
    return 0;
  }
  	ROS_INFO("AUTO.LAND");
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    set_mode_client.call(offb_set_mode);
  return 0;
}


