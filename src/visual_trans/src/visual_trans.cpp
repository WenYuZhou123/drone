/******************************************************************************
作者：王蔚
功能：识别多个物体或者从多个角度识别同一个物体，识别后返回相对坐标值
******************************************************************************/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <string>
using namespace std;


//创建了4个坐标系名称
string  object[4] = {"object_1","object_2","object_3","object_4"};

//创建了坐标转换，用于转换不同坐标系转转换出来的位置信息
tf::StampedTransform transform_table[4] = {};
       
//上一次x位置数据，用于判断是否检测到目标，检测到目标后，则坐标系位置产生变化，此处选择x方向位置进行判断，也可采用Y或者Z都可以
double last_pose_table[4]    = {0,0,0,0} ;

//roll,pitch,yaw是一组数据
double angle_table[4][3] = {{0,0,0}};


//x,y,z是一组位置数据
double pose_table[4][3]  = {{0,0,0}};

//检测到的物体坐标值
double position_detec_x = 0;
double position_detec_y = 0;
double position_detec_z = 0;

//参数m用于设置执行几个样本的识别，若提前采样了四个方向的图片用于识别，则m取4即可，根据数据定义，m应当取值0-4
void visual_trans( string object[4], double last_pose_table[4], double pose_table[4][3], double angle_table[4][3],int m)
{
		// 创建tf的监听器
	        tf::TransformListener listener;		
		// 获取turtle1与turtle2坐标系之间的tf数据
                ROS_INFO("111");
                for(int i=0;i<m;i++)
		{
				try
				{
					listener.waitForTransform("d435_link", object[i], ros::Time(0), ros::Duration(0.3));
					listener.lookupTransform( "d435_link", object[i], ros::Time(0), transform_table[i]);
				        //last_pose_table[i] = pose_table[i][0];
					pose_table[i][0]      = transform_table[i].getOrigin().x();
					pose_table[i][1]      = transform_table[i].getOrigin().y();
		  			pose_table[i][2]      = transform_table[i].getOrigin().z();
					tf::Matrix3x3(transform_table[i].getRotation()).getEulerYPR(angle_table[i][0], angle_table[i][1], angle_table[i][2]);
            				
					///ROS_INFO("i  = %d\r", i);
				        //ROS_INFO("last_pose_table_x = %f\r", last_pose_table[i]);
				       // ROS_INFO("pose_table_x = %f\r\n\r\n", pose_table[i][0]);
					if(last_pose_table[i] != pose_table[i][0])
					{    
						position_detec_x = pose_table[i][0];
						position_detec_y = pose_table[i][1];
						position_detec_z = pose_table[i][2];

						ROS_INFO("i  = %d\r", i);
						ROS_INFO("last_pose_table_x     = %f\r",   last_pose_table[i]);
						ROS_INFO("position_detec_x      = %f\r",   position_detec_x);
						ROS_INFO("position_detec_y      = %f\r",   position_detec_y);
						ROS_INFO("position_detec_z      = %f\r\n", position_detec_z);
						last_pose_table[i] = pose_table[i][0];
					        break;
					}
					
				}
				catch (tf::TransformException &ex) 
				{
					ROS_ERROR("%s",ex.what());
					ros::Duration(0.2).sleep();
					continue;
				}			
		}		

}
int main(int argc, char** argv)
{
	// 初始化ROS节点
	ros::init(argc, argv, "visual_trans");

        // 创建节点句柄
	ros::NodeHandle node;

	ros::Rate rate(10.0);
	while (node.ok())
	{
		visual_trans(object, last_pose_table,  pose_table,  angle_table, 2);
		rate.sleep();
	}
	return 0;
};
