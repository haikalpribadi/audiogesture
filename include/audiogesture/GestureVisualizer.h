/* 
 * File:   GestureVisualization.h
 * Author: haikalpribadi
 *
 * Created on 26 March 2014, 16:49
 */

#ifndef GESTUREVISUALIZER_H
#define	GESTUREVISUALIZER_H

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;

class GestureVisualizer {
public:
    GestureVisualizer();
    
private:
    ros::NodeHandle node;
    ros::Publisher marker_pub;
    ros::Publisher markerarray_pub;
    ros::Subscriber gesture_sub;
    
    string gestureTopic;
    int columns;
    int rows;
    int scale;
    
    int shape;
    
    void gestureCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    visualization_msgs::Marker createMarker(int id, int x, int y, int z);
};

#endif	/* GESTUREVISUALIZER_H */
