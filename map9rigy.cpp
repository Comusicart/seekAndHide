#include <ros/ros.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "std_msgs/String.h"

#define MAP_SIZE 800
#define CHECK_SIZE 50	//현재 스캔값으로 바꿀 범위

using namespace cv;

// Global variable
boost::mutex mutex[3];
nav_msgs::Odometry g_odom;
sensor_msgs::LaserScan g_scan;
Vec3d stamp_xyz;

int map[MAP_SIZE][MAP_SIZE];	// 0->빈곳, 1->turn이 false일 때 스캔 값, 2->turn이 true일 때 스캔 , 3->경로, 4->마커
bool turn;
bool marking;
std_msgs::String whatMarker;

void odomMsgCallback(const nav_msgs::Odometry &msg) {
	// receive a '/odom' message with the mutex값
	mutex[0].lock(); {
		g_odom = msg;
	} mutex[0].unlock();
}

void stampCallback(const std_msgs::String::ConstPtr& msg) {
	marking = true;
	if(!( (msg->data).empty() ) ){
		ROS_INFO("%s", msg->data.c_str());
		whatMarker.data = msg->data.c_str();
	}
}

void scanMsgCallback(const sensor_msgs::LaserScan& msg) {
	// receive a '/odom' message with the mutex
	mutex[1].lock(); {
		g_scan = msg;
	} mutex[1].unlock();
}

// callback function
void draw_marker(Mat &display) {
	int x = 100 * stamp_xyz[0] + 400;
	int y = 100 * stamp_xyz[1] + 400;	

	if(strcmp(whatMarker.data.c_str(), "one") == 0)
		circle(display, Point(x, y), 8, CV_RGB(255, 0, 0));
	else if(strcmp(whatMarker.data.c_str(), "two") == 0)
		circle(display, Point(x, y), 8, CV_RGB(0, 255, 0));
	else if(strcmp(whatMarker.data.c_str(), "three") == 0)
		circle(display, Point(x, y), 8, CV_RGB(0, 0, 255));
	mutex[2].lock(); {
		if(x >= 0 && x <= MAP_SIZE && y >= 0 && y < MAP_SIZE)
			map[x][y] = 4;
	} mutex[2].unlock();
}

void drawMap(Mat &display) {
	mutex[2].lock(); {
		//이차원 map배열값을 돌면서 openCV화면에 circle로 그린다.
		for(int raw = 0 ; raw < MAP_SIZE ; raw++)
		{
			for(int col = 0 ; col < MAP_SIZE ; col++)
			{
				if(map[raw][col] == 1)
				{
					circle(display, Point(raw, col), 2, CV_RGB(255,255,0));
					//turn이 true이면 1값을 2로 바꾸어 준다.
					if(turn == true)
						map[raw][col] = 2;
				}
				else if(map[raw][col] == 2)
				{
					circle(display, Point(raw, col), 2, CV_RGB(255,255,0));
					//turn이 false이면 2값을 1로 바꾸어 준다.
					if(turn == false)
						map[raw][col] = 1;
				}
				//turtlebot 이동 경로를 그려준다.
				else if(map[raw][col] == 3)
				{
					circle(display, Point(raw, col), 2, CV_RGB(0,255,0));
				}
				else if(map[raw][col] == 4)
				{
					circle(display, Point(raw, col), 8, CV_RGB(255, 255, 255));
				}
			}
		}
		// turn을 바꾸어 준다.
		turn = !turn;	
	} mutex[2].unlock();
}

void inputMap(Mat &display, vector<Vec3d> &laserScanXY, double dMaxDist) {
	// laserScan size만큼 값을 map배열에 넣어 준다.
	for(int i = 0 ; i < laserScanXY.size() ; ++i) {
		Vec3d scanVec = laserScanXY[i];

		int x = 100 * scanVec[0] + 400;
		int y = 100 * scanVec[1] + 400;		//값을 키우고 중앙으로 오게 한다.

		mutex[2].lock(); {
			// CHECK_SIZE만큼 이전 스캔값을 현재 스캔값으로 바꾸어 준다.
			for(int raw = x-CHECK_SIZE/2 ; raw < x+CHECK_SIZE/2 ; raw++)
			{
				for(int col = y-CHECK_SIZE/2 ; col < y+CHECK_SIZE/2 ; col++)
				{
					if(raw >= 0 && raw <= MAP_SIZE && col >= 0 && col < MAP_SIZE && map[x][y] != 3)
					{
						if(turn == false && map[raw][col] == 2)
							map[raw][col] == 0;
						else if(turn == true && map[raw][col] == 1)
							map[raw][col] == 0;
					}
				}
			}
			if(x >= 0 && x <= MAP_SIZE && y >= 0 && y < MAP_SIZE && map[x][y] != 3)
			{
				if(turn == false)
					map[x][y] = 1;
				else
					map[x][y] = 2;
			}
		} mutex[2].unlock();
	}
	//맵을 openCV로 그려주는 함수 실행
	drawMap(display);
}

void convertOdom2XYZRPY(nav_msgs::Odometry &odom, Vec3d &xyz, Vec3d &rpy) {
	// 이동 저장
	xyz[0] = odom.pose.pose.position.x;
	xyz[1] = odom.pose.pose.position.y;
	xyz[2] = odom.pose.pose.position.z;

	// 회전 저장
	tf::Quaternion rotationQuat = tf::Quaternion(odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w);
	tf::Matrix3x3(rotationQuat).getEulerYPR(rpy[2], rpy[1], rpy[0]);

	//이동한 turtlebot의 x, y 값을 map배열에 넣어준다.
	mutex[2].lock(); {
		int x = 100 * xyz[0] + 400;
		int y = 100 * xyz[1] + 400;	//값을 키우고 중앙으로 오게 한다.
		if(x >= 0 && x <= MAP_SIZE && y >= 0 && y < MAP_SIZE)
		{
			map[x][y] = 3;
		}
	} mutex[2].unlock();
	
	stamp_xyz[0] = xyz[0];
	stamp_xyz[1] = xyz[1];
}

void transform(vector<Vec3d> &laserScanXY, double x, double y, double theta) {
	Vec3d newPt;
	double cosTheta = cos(theta);
	double sinTheta = sin(theta);
	int nRangeSize = (int)laserScanXY.size();

	for(int i = 0 ; i < nRangeSize ; i++) {
		newPt[0] = cosTheta*laserScanXY[i][0] + -1.*sinTheta*laserScanXY[i][1] + x;
		newPt[1] = sinTheta*laserScanXY[i][0] + cosTheta*laserScanXY[i][1] + y;
		newPt[2];
		laserScanXY[i] = newPt;
	}
}

void convertScan2XYZs(sensor_msgs::LaserScan& lrfScan, vector<Vec3d> &XYZs) {
    int nRangeSize = (int)lrfScan.ranges.size();
    XYZs.clear();
    XYZs.resize(nRangeSize);

    for(int i=0; i<nRangeSize; i++) {
        double dRange = lrfScan.ranges[i];

        if(isnan(dRange)) {
            XYZs[i] = Vec3d(0., 0., 0.);
        } else {
            double dAngle = lrfScan.angle_min + i*lrfScan.angle_increment;
            XYZs[i] = Vec3d(dRange*cos(dAngle), dRange*sin(dAngle), 0.);
        }
    }
}

void initGrid(Mat &display, int nImageSize) {
    const int nImageHalfSize = nImageSize/2;
    const int nAxisSize = nImageSize/16;
    const Vec2i imageCenterCooord = Vec2i(nImageHalfSize, nImageHalfSize);
    display = Mat::zeros(nImageSize, nImageSize, CV_8UC3);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0]+nAxisSize, imageCenterCooord[1]), Scalar(0, 0, 255), 2);
    line(display, Point(imageCenterCooord[0], imageCenterCooord[1]), Point(imageCenterCooord[0], imageCenterCooord[1]+nAxisSize), Scalar(0, 255, 0), 2);
}

int main(int argc, char **argv) {
	// Initialize the ROS system
	ros::init(argc, argv, "map_building");

	// Create subscriber objects
	ros::NodeHandle nh_scan;
    	ros::Subscriber subScan = nh_scan.subscribe("/scan", 10, &scanMsgCallback);

	ros::NodeHandle nh_stamp;
	ros::Subscriber subStamp = nh_stamp.subscribe("Stamp", 10, &stampCallback);

	ros::NodeHandle nh_odom;
	ros::Subscriber subOdom = nh_odom.subscribe("/odom", 10, &odomMsgCallback);

	// Display buffer
	Mat display;
	initGrid(display, MAP_SIZE);

	// Odometry buffer
	nav_msgs::Odometry odom;

	// Scan buffer
	sensor_msgs::LaserScan scan;

	// 이동 및 회전 정보
	Vec3d xyz, rpy;

	//LRF scan 정보
	vector<Vec3d> laserScanXY;

	//Mat distance for grid
	const double dGridMaxDist = 4.5;

	turn = false;
	marking = false;

	for(int i = 0 ; i < MAP_SIZE ; i++)
		for(int j = 0 ; j < MAP_SIZE ; j++)
			map[i][j] = 0;

	// 2D 영상좌표계에서 top-view 방식의 3차원 월드좌표계로 변환
       	transpose(display, display);  // X-Y축 교환
       	flip(display, display, 0);  // 수평방향 반전
       	flip(display, display, 1);  // 수직방향 반전

	ros::Rate rate(2);

	// main loop
	while(ros::ok()) {
		// callback 함수 call!
		ros::spinOnce();

		// receive the global '/odom' message with the mutex
		mutex[0].lock(); {
			odom = g_odom;
		} mutex[0].unlock();

		// odom으로부터 이동 및 회전정보 획득한다.
		convertOdom2XYZRPY(odom, xyz, rpy);

		// receive the global '/scan' message with the mutex
		mutex[1].lock(); {
			scan = g_scan;
		} mutex[1].unlock();

		// scan으로부터 Cartesian X-Y scan 획득한다.
		convertScan2XYZs(scan, laserScanXY);

		//laserScan을 월드좌표계로 변환
		transform(laserScanXY, xyz[0], xyz[1], rpy[2]);

		// 현재 상황을 draw할 display 이미지를 생성한다.
	        initGrid(display, MAP_SIZE);

		//map배열에 값을 넣고 openCV로 출력한다.
		inputMap(display, laserScanXY, dGridMaxDist);	

		if(marking)
		{
			draw_marker(display);
			marking = false;
		}

		// image 출력
		imshow("KNU ROS Lecture >> turtle_position_lrf_view", display);

		// 사용자의 키보드 입력을 받음!
	        int nKey = waitKey(30) % 255;

        	if(nKey == 27) {
        	    // 종료
        	    break;
        	}
		rate.sleep();
	}
}
