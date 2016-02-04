#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iomanip>
#include <boost/thread/mutex.hpp>
#include "std_msgs/String.h"
#include <sstream>

using namespace cv;
using namespace std;

std_msgs::String message;
std::stringstream ss;
boost::mutex mark_mutex;
ros::Publisher sign_marker;

double angle(  Point pt1,  Point pt2,  Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

void find_squares( Mat& image,  vector< vector< Point> >& squares)
{

    // blur will enhance edge detection
    GaussianBlur(image, image, Size(5,5), 1 ,1);

    Mat gray0;
    cvtColor(image,gray0, CV_BGR2GRAY);

    Mat gray(image.size(), CV_8U);

    vector< vector< Point> > contours;

    gray = gray0 >= 100;

    imshow("binary", gray);
     // Find contours and store them in a list
    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

     // Test contours
      vector< Point> approx;
     for (size_t i = 0; i < contours.size(); i++)
     {
             // approximate contour with accuracy proportional
             // to the contour perimeter
              approxPolyDP( Mat(contours[i]), approx,  arcLength( Mat(contours[i]), true)*0.02, true);

             // Note: absolute value of an area is used because
             // area may be positive or negative - in accordance with the
             // contour orientation
             if (approx.size() == 4 &&
                     fabs(contourArea( Mat(approx))) > 1000 &&
                     isContourConvex( Mat(approx)))
             {
                     double maxCosine = 0;

                     for (int j = 2; j < 5; j++)
                     {
                             double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                             maxCosine = MAX(maxCosine, cosine);
                     }

                     if (maxCosine < 0.3)
                             squares.push_back(approx);
             }
     }

}

void find_triangle( Mat& image,  vector< vector< Point> >& triangles)
{
    // blur will enhance edge detection
    GaussianBlur(image, image, Size(5,5), 1 ,1);

    Mat gray0;
    cvtColor(image,gray0, CV_BGR2GRAY);

    Mat gray(image.size(), CV_8U);

    vector< vector< Point> > contours;

    gray = gray0 >= 100;

    imshow("binary", gray);
     // Find contours and store them in a list
    findContours(gray, contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);

     // Test contours
      vector< Point> approx;
     for (size_t i = 0; i < contours.size(); i++)
     {
             // approximate contour with accuracy proportional
             // to the contour perimeter
              approxPolyDP( Mat(contours[i]), approx,  arcLength( Mat(contours[i]), true)*0.02, true);

             // Note: absolute value of an area is used because
             // area may be positive or negative - in accordance with the
             // contour orientation
             if (approx.size() == 3 &&
                     fabs(contourArea( Mat(approx))) > 1000 &&
                     isContourConvex( Mat(approx)))
             {
                     double maxCosine = 0;

                     for (int j = 1; j < 3; j++)
                     {
                             double cosine = fabs(angle(approx[j%3], approx[j-1], approx[j]));
                             maxCosine = MAX(maxCosine, cosine);
                     }

                     if (maxCosine < 0.3)
                             triangles.push_back(approx);
             }
     }

}


// A callback function. Executed eack time a new pose message arrives.
void poseMessageReceivedRGB(const sensor_msgs::Image& msg) {

    ROS_INFO("seq = %d / width = %d / height = %d / step = %d", msg.header.seq, msg.width, msg.height, msg.step);
    ROS_INFO("encoding = %s", msg.encoding.c_str());

    Mat img = Mat(msg.height, msg.width, CV_8UC3);
    
    memcpy(img.data, &msg.data[0], sizeof(unsigned char)*msg.data.size());
	Mat img_origin = img.clone();

	vector< vector< Point> > squares;
	vector< vector< Point> > triangles;
	find_squares(img, squares);
	find_triangle(img, triangles);

	
	if(squares.size() == 5){
		for(int i=0;i<5;i++){
			vector<Point> temp_square = squares[i];
			for(int j=0;j<4;j++){
				line(img, temp_square[j], temp_square[(j+1)%4], Scalar(200,200,100), 3, CV_AA);
			}
		}
		mark_mutex.lock();{
			ss << "one";
			message.data = ss.str();
		}mark_mutex.unlock();
	
		sign_marker.publish(msg);	
	}

	if((triangles.size() == 3) && (squares.size() == 2)){
		for(int i=0;i<3;i++){
			vector<Point> temp_square = triangles[i];
			for(int j=0;j<3;j++){
				line(img, temp_square[j], temp_square[(j+1)%3], Scalar(100,0,250), 3, CV_AA);
			}
		}
		for(int i=0;i<2;i++){
			vector<Point> temp_square = squares[i];
		
			for(int j=0;j<4;j++){
				line(img, temp_square[j], temp_square[(j+1)%4], Scalar(100,0,250), 3, CV_AA);
			}
		}

		mark_mutex.lock();{
			ss << "two";
			message.data = ss.str();
		}mark_mutex.unlock();
	
		sign_marker.publish(msg);		
	}


	
	if((triangles.size() == 2) && (squares.size() == 1)){
		for(int i=0;i<2;i++){
			vector<Point> temp_square = triangles[i];
			for(int j=0;j<3;j++){
				line(img, temp_square[j], temp_square[(j+1)%3], Scalar(0,250,0), 3, CV_AA);
			}
		}

		vector<Point> temp_square = squares[0];
		for(int j=0;j<4;j++){
			line(img, temp_square[j], temp_square[(j+1)%4], Scalar(0,250,0), 3, CV_AA);
		}

		mark_mutex.lock();{
			ss << "three";
                        message.data = ss.str();
		}mark_mutex.unlock();
	
		sign_marker.publish(msg);
	}



	imshow("img_origin",img_origin);
	imshow("pattern", img);
	waitKey(30);
}

int main(int argc, char **argv)
{
    // Initialize the ROS system
    ros::init(argc, argv, "turtle_kinect_image_view");
    ros::NodeHandle nh;

        // Create a subscriber object
    ros::Subscriber subRgb = nh.subscribe("/camera/rgb/image_color", 10, &poseMessageReceivedRGB);
    sign_marker = nh.advertise<std_msgs::String>("goToPattern", 1000);
    // Let ROS take over
    ros::spin();

    return 0;
}

