/* 
 * File:   GestureVisualization.h
 * Author: haikalpribadi
 *
 * Created on 26 March 2014, 16:49
 */

#ifndef GESTUREVISUALIZER_H
#define	GESTUREVISUALIZER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "audiogesture/FeatureVector.h"
#include "audiogesture/GestureVector.h"
#include "audiogesture/SetDimensions.h"

using namespace std;

class GestureVisualizer {
public:
    GestureVisualizer();
    
private:
    ros::NodeHandle node;
    ros::Publisher marker_pub;
    ros::Publisher markerarray_pub;
    ros::Subscriber gesture_sub;
    ros::Subscriber feature_sub;
    
    string gestureTopic;
    string featureTopic;
    int columns;
    int rows;
    int scale;
    double duration;
    double amp;
    
    int shape;
    
    void gestureCallback(const audiogesture::GestureVector::ConstPtr& msg);
    void featureCallback(const audiogesture::FeatureVector::ConstPtr& msg);
    visualization_msgs::Marker createMarker(int id, int x, int y, double z);
};

#endif	/* GESTUREVISUALIZER_H */
