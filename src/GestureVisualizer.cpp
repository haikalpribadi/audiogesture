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
        ROS_INFO("GestureVisualizer started listening to /gesture_vector and publishing markers");
    } else {
        ROS_ERROR("Please set the gesture_topic parameter for GestureVisualizer");
        ros::requestShutdown();
    }
    
    marker_pub = node.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    markerarray_pub = node.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 0);
    
    shape = visualization_msgs::Marker::CUBE;
    columns = 8;
    rows = 4; 
    scale = 2;
}


void GestureVisualizer::gestureCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    visualization_msgs::MarkerArray markerarray;
    int id;
    for(int i=0; i<rows; i++) {
        for(int j=0; j<columns; j++) {
            id = j + i*columns;
            markerarray.markers.push_back(createMarker(id, i*scale, j*scale, msg->data[id]));
        }
    }
    // Publish the marker
    markerarray_pub.publish(markerarray);
}

visualization_msgs::Marker GestureVisualizer::createMarker(int id, int x, int y, int z) {
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
    marker.color.r = float(x/(rows));
    marker.color.g = float(y/(columns));
    marker.color.b = 1 - float(x/(rows) + y/(columns));
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    return marker;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureVisualizer");
    
    GestureVisualizer visualizer;
    
    ros::spin();
    
    return 0;
}