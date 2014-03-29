/* 
 * File:   GestureOutput.cpp
 * Author: haikalpribadi
 * 
 * Created on 29 March 2014, 20:51
 */

#include "GestureOutput.h"

GestureOutput::GestureOutput() {
    if(node.getParam("gesture_range", range)) {
        ROS_INFO("GestureOutput is publishing in the range of 0 to %d", range);
    } else {
        range = 1024;
        ROS_INFO("GestureOutput is publishing in the range of 0 to %d", range);
    }
    gestureOutput_sub = node.subscribe("gesture_output", 1000,
                                       &GestureOutput::gestureOutputCallback, this);
    oscSend_pub = node.advertise<std_msgs::Int32MultiArray>("osc_send_vector",1000);
    min = 0;
    max = 0;
}

void GestureOutput::gestureOutputCallback(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    vector<int> gesture(msg->data.size());
    double new_max = *max_element(msg->data.begin(), msg->data.end());
    double new_min = *min_element(msg->data.begin(), msg->data.end());
    max = new_max > max ? new_max : max;
    min = new_min < min ? new_min : min;
    
    for(int i=0; i<gesture.size(); i++) {
        gesture[i] = (int)((max-min)==0 ? 0 : ((msg->data[i]-min)/(max-min))*range);
    }
    
    // This is the part where we right the vector out to the ethernet
    // *********************************************** //
    
    // TRYING TO SEND TO OSC SENDER THROUGH UDP
    std_msgs::Int32MultiArray osc;
    for(int i=0; i<gesture.size(); i++) {
        osc.data.push_back(gesture[i]);
    }
    oscSend_pub.publish(osc);
    
    // *********************************************** //
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureOutput");
    
    GestureOutput output;
    
    ros::spin();
    
    return 0;
}