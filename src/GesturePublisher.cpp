/* 
 * File:   SerialDataPublisher.cpp
 * Author: haikalpribadi
 * 
 * Created on 10 March 2014, 22:25
 */

#include "GesturePublisher.h"

GesturePublisher::GesturePublisher() {
    gestureVector_pub = node.advertise<std_msgs::Int32MultiArray>("gesture_vector", 1000);
    
    gestureVector0_sub = node.subscribe("serial_data_0", 1000,
                                        &GesturePublisher::gestureVector0Callback, this);
    gestureVector1_sub = node.subscribe("serial_data_1", 1000,
                                        &GesturePublisher::gestureVector1Callback, this);
    
    ROS_INFO("GesturePublisher has subscribed to /serial_data_0 and /serial_data_1 and publish to /gesture_vector");
}

void GesturePublisher::gestureVector0Callback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    vector<int> gesture(msg->data);
    gesture0queue.push(gesture);
}

void GesturePublisher::gestureVector1Callback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    vector<int> gesture(msg->data);
    gesture1queue.push(gesture);
}

void GesturePublisher::publishGestureVector() {
    while(gesture0queue.size()>0 && gesture1queue.size()>0) {
        std_msgs::Int32MultiArray gesture;
        gesture.data.insert(gesture.data.end(), 
                            gesture0queue.front().begin(), 
                            gesture0queue.front().end());
        gesture.data.insert(gesture.data.end(), 
                            gesture1queue.front().begin(), 
                            gesture1queue.front().end());
        gesture0queue.pop();
        gesture1queue.pop();
        
        gestureVector_pub.publish(gesture);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "SerialDataPublisher");
    
    GesturePublisher serial;
    ros::Rate rate(100);
    
    while(ros::ok()) {
        ros::spinOnce();
        ros::spinOnce();
        serial.publishGestureVector();
        rate.sleep();
    }
}

