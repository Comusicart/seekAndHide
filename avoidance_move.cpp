#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/Twist.h>
#include <boost/thread/mutex.hpp>
#include <iostream>
#include <cstdlib>

#define RANDOM_LINEAR (double)rand() / ((double)(RAND_MAX)+(double)(1)) / (double)(2);
#define RANDOM_ANGULAR (double)rand() / ((double)(RAND_MAX)+(double)(1));

boost::mutex mutex;
sensor_msgs::LaserScan g_scan;
geometry_msgs::Twist cmdvel;

double min_range_old;
bool near;
int leftRightCount;

// callback function
void scanMsgCallback(const sensor_msgs::LaserScan& msg)
{
	mutex.lock(); {
		g_scan = msg;
	} mutex.unlock();
//	printf("callback function\n");
}

// move to some where
int moveToSomeWhere(sensor_msgs::LaserScan& lrfScan)
{
	int nRangeSize = (int)lrfScan.ranges.size();	//laserscan 값수
	printf("%d\n", nRangeSize);
	if(nRangeSize == 0)
	{
		cmdvel.linear.x = RANDOM_LINEAR;
		if(rand()%2 == 0)
		{
			cmdvel.angular.z = RANDOM_ANGULAR;
		}			
		else
		{
			cmdvel.angular.z = -1*RANDOM_ANGULAR;
		}		
		printf("first random!!\n");
		return 0;
	}

	double min_range = 0;
	double min_range_angle = 0;		//가장 작은 range 값을 찾기위한 초기화

	min_range = 10;
	
	for(int i=0 ; i < nRangeSize; i++) //가장 작은 range 값 찾기
	{
		if(!isnan(lrfScan.ranges[i]))
		{
			printf("aaa%lf\n", lrfScan.ranges[i]);
		}
		if(lrfScan.ranges[i] < min_range && !isnan(lrfScan.ranges[i]))
		{
			min_range = lrfScan.ranges[i];
			min_range_angle = i;
		}
	}

    printf("minimum range is [%f] at an angle of [%f] count [%d] \n", min_range, min_range_angle, leftRightCount);

	if(isnan(min_range))	//nan일때
	{
		if(min_range_old > 0 && min_range_old <= 2)
		{
			near = true;
			printf("near!\n");
		}
		else if(min_range_old > 2)
		{
			near = false;
			printf("not near!!\n");
		}

		if(near)	//너무 가까이 있어서 nan
		{
			cmdvel.linear.x = 0;
			cmdvel.angular.z = 0.75;
			printf("non=1 back\n");
		}
		else		//앞에 너무 아무것도 없어서 nan
		{
			cmdvel.linear.x = 0.3;
			if(rand()%2 == 0)
			{
				cmdvel.angular.z = RANDOM_ANGULAR;
			}			
			else
			{
				cmdvel.angular.z = -1*RANDOM_ANGULAR;
			}
			printf("random!!\n");
		}
	}
	else if(min_range <= 0.6)
	{
		if(min_range_angle < nRangeSize/2)	//왼쪽으로 돌기 
		{
			cmdvel.angular.z = 0.3;
			cmdvel.linear.x = 0;
			printf("left\n");
			leftRightCount++;
		}
		else					//오른쪽으로 돌기
		{
			cmdvel.angular.z = -0.3;
			cmdvel.linear.x = 0;
			printf("right\n");
			leftRightCount++;
		}
		
		if(leftRightCount > 30)
		{
			cmdvel.angular.z = 0.75;
			cmdvel.linear.x = 0;
			printf("stucked!!!!\n");
			leftRightCount = 0;
		}
	}

	else	// 랜덤 주행
	{
		cmdvel.linear.x = RANDOM_LINEAR;
		if(rand()%2 == 0)
		{
			cmdvel.angular.z = RANDOM_ANGULAR;
		}			
		else
		{
			cmdvel.angular.z = -1*RANDOM_ANGULAR;
		}
		leftRightCount = 0;
	        printf("%lf, %lf random move!!!\n", cmdvel.linear.x, cmdvel.angular.z);
	}

	if(isnan(min_range))
	{
		printf("[%lf] min_range_old\n", min_range_old);
	}
	else
	{
		min_range_old = min_range;
		printf("[%lf] min_range_old\n", min_range_old);
	}

	return 0;
}

int main(int argc, char **argv)
{
	// Initialize the ROS system
	ros::init(argc, argv, "avoidance_move");
	ros::NodeHandle moveHandle;
	ros::NodeHandle scanHandle;

	ros::Publisher movepub = moveHandle.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",10);

	// Create subscriber objects
    ros::Subscriber subScan = scanHandle.subscribe("/scan", 10, &scanMsgCallback);

	// Scan buffer
	sensor_msgs::LaserScan scan;

	min_range_old = -1;
	near = false;
	leftRightCount = 0;

	//main loop
    ros::Rate rate(2);
	while(ros::ok()) {
		// callback 함수 call!
		ros::spinOnce();

		// receive the global '/scan' message with the mutex
		mutex.lock(); {
			scan = g_scan;
		} mutex.unlock();

		// scan으로 부터 움직일 방향 정하기
		moveToSomeWhere(scan);

		movepub.publish(cmdvel);

		rate.sleep();
	}

	return 0;
}
