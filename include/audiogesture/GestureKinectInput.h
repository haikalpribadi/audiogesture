/* 
 * File:   GestureKinectInput.h
 * Author: haikalpribadi
 *
 * Created on 07 May 2014, 15:56
 */

#ifndef GESTUREKINECTINPUT_H
#define	GESTUREKINECTINPUT_H

#include <math.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Empty.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "audiogesture/GestureVector.h"
#include "audiogesture/SetMargin.h"

using namespace std;

class GestureKinectInput {
public:
    GestureKinectInput();
    
private:
    ros::NodeHandle node;
    ros::Publisher gesture_pub;
    ros::Subscriber depth_sub;
    ros::Subscriber calibrate_sub;
    ros::Subscriber margin_sub;
    ros::Publisher image_pub;
    
    int counter;
    int filterSize;
    int filterSizeN;
    int sampleSize;
    int x_min;
    int x_max;
    int y_min;
    int y_max;
    int z_min;
    int z_max;
    float sigma;
    bool smoothing;
    bool calibrate;
    bool calibrated;
    int calibrationSampleSize;
    vector<vector<float> > calibrationPoints;
    vector<float> offsetPoints;
    vector<float> gaussianFilter;
    
    //void imageCb(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    void calibrateCallback(const std_msgs::Empty::ConstPtr& msg);
    void calibrateOffset();
    void marginCallback(const audiogesture::SetMargin::ConstPtr& msg);
};

#endif	/* GESTUREKINECTINPUT_H */

