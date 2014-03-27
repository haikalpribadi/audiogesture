/* 
 * File:   GestureTest.cpp
 * Author: haikalpribadi
 * 
 * Created on 06 March 2014, 14:44
 */

#include "GestureTest.h"

GestureTest::GestureTest() {
    gesture_pub = node.advertise<std_msgs::Int32MultiArray>("osc_send_vector",1000);
    ROS_INFO("GestureTest has started publishing dummy data");
}

void GestureTest::sendData(std_msgs::Int32MultiArray& msg) {
    gesture_pub.publish(msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureTestData");
    GestureTest gesture;
    ros::Rate rate(10);
    
    std_msgs::Int32MultiArray msg;
    int val = 0;
    
    while(ros::ok()) {
        for(int i=0; i<32; i++) {
            msg.data.push_back(val);
            
        }
        val = val < 5 ? ++val : -5;
        gesture.sendData(msg);
        msg.data.clear();
        rate.sleep();
    }
}