#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cmath>
#include<ctime>
#include <chrono>



#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include "geometry_msgs/Point.h"
#include <nav_msgs/Odometry.h>

#include "opencv2/highgui.hpp"
#include "opencv2/tracking/tracker.hpp"
#include <opencv2/bgsegm.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <fstream> 
#include "std_msgs/String.h"
#include <sstream>
#include <string.h>

using namespace cv;

class prueba
{
private:
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void cloudCallback(const sensor_msgs::PointCloud2Ptr& msg);
    void ekfCallback(const nav_msgs::Odometry& odom_msg);

    ros::NodeHandle n_;
    ros::Subscriber subDepth;
    ros::Subscriber subImage;
    ros::Subscriber subCloud;
    ros::Subscriber subEKF;
    ros::Publisher odom_pub;
    ros::Publisher depth_pub;
    ros::Publisher angle_pub;
    
    Ptr<TrackerKCF> tracker;
   	Ptr<TrackerKCF> trackerFront = TrackerKCF::create(); // Front tracker
    Ptr<BackgroundSubtractor> pBackSub;


    int width, height;
    cv_bridge::CvImagePtr cv_image, cv_image_depth;
    Mat mat_image, mat_image_depth;
    float* depths;
    float* confidence;
    int pos_persona;
    bool flag_tracking, flag_histo, flag_cloud;
    Rect2d bbox;
    unsigned int* hist_confidence = (unsigned int *)calloc(100, sizeof(int));
    double desv;
    float media;
    int posX, posY;
    int pos_cloud;
    float media_depth = 0;
    float desv_depth = 0;
    float aux_profundidad = 0;
    float aux_angulo = 0;
    unsigned int cont_cloud = 0;
    std::vector <unsigned int> posiciones_x;
    std::vector <unsigned int> posiciones_y;
    float media_profundidad = 0;
    bool flag_clock = false;

public:
    prueba(ros::NodeHandle n);
    ~prueba();
    int main();

};
