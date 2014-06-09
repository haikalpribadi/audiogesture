/* 
 * File:   GestureVisualization.cpp
 * Author: haikalpribadi
 * 
 * Created on 26 March 2014, 16:49
 */

#include "GestureVisualizer.h"

GestureVisualizer::GestureVisualizer() {
    if(node.getParam("gesture_topic", gestureTopic)) {
        gesture_sub = node.subscribe(gestureTopic, 1000, &GestureVisualizer::gestureCallback, this);
        ROS_INFO("GestureVisualizer started listening to /%s and publishing markers", gestureTopic.c_str());
    } else if(node.getParam("feature_topic", featureTopic)) {
        feature_sub = node.subscribe(featureTopic, 1000, &GestureVisualizer::featureCallback, this);
        ROS_INFO("GestureVisualizer started listening to /%s and publishing markers", featureTopic.c_str());
    } else {
        ROS_ERROR("Please set the gesture_topic parameter for GestureVisualizer");
        ros::requestShutdown();
    }
    
    amp = 1.0;
    if(node.getParam("transform_mult", amp)) {
        ROS_INFO("DataVisualizer using amp: %f", amp);
    }
    
    if(node.getParam("visualizer_scale", scale)) {
        ROS_INFO("GestureVisualizer set scale: ", scale);
    } else {
        scale = 2;
    }
    
    node.param("sample_duration", duration, 5.0);
    duration = 2 / duration;
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 1000);
    markerarray_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 1000);
    
    shape = visualization_msgs::Marker::CUBE;
}

void GestureVisualizer::featureCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    rows = 1;
    columns = msg->data.size();
    
    visualization_msgs::MarkerArray markerarray;
    int id;
    for(int i=0; i<rows; i++) {
        for(int j=0; j<columns; j++) {
            id = j + i*columns;
            markerarray.markers.push_back(createMarker(id, i*scale, j*scale, msg->data[id]*amp));
        }
    }
    // Publish the marker
    markerarray_pub.publish(markerarray);
}


void GestureVisualizer::gestureCallback(const audiogesture::GestureVector::ConstPtr& msg) {
    rows = msg->height;
    columns = msg->width;
    if(rows*columns != msg->data.size()) {
        ROS_ERROR("GestureVisualizer not receiving data in correct size of %d x %d. Receiving legnth: %d", rows, columns, msg->data.size());
        return;
    }
    visualization_msgs::MarkerArray markerarray;
    int id;
    for(int i=0; i<rows; i++) {
        for(int j=0; j<columns; j++) {
            id = j + i*columns;
            markerarray.markers.push_back(createMarker(id, i*scale, j*scale, msg->data[id]*amp));
        }
    }
    // Publish the marker
    markerarray_pub.publish(markerarray);
}

visualization_msgs::Marker GestureVisualizer::createMarker(int id, int x, int y, double z) {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/gesture";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "gesture_positions";
    marker.id = id;

    // Set the marker type. Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = (float)x/rows + (float)1/rows;
    marker.color.b = (float)y/columns + (float)1/columns;
    marker.color.g = 1 - ((marker.color.r + marker.color.b)/3);
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(duration);
    
    return marker;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureVisualizer");
    
    GestureVisualizer visualizer;
    
    ros::spin();
    
    return 0;
}