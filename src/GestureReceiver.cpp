/* 
 * File:   GestureReceiver.cpp
 * Author: haikalpribadi
 * 
 * Created on 05 March 2014, 19:56
 */

#include "GestureReceiver.h"

GestureReceiver::GestureReceiver() {
    gestureVector_sub = node.subscribe("osc_receive_vector", 1000,
                                       &GestureReceiver::gestureVectorCallback, this);
    gestureMessage_sub = node.subscribe("osc_receive_message", 1000,
                                        &GestureReceiver::gestureMessageCallback, this);
}

void GestureReceiver::gestureMessageCallback(const std_msgs::String::ConstPtr& msg) {
    
}

void GestureReceiver::gestureVectorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {

}



int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureReceiver");
    
    GestureReceiver gesture;
    
    ros::spin();
}
