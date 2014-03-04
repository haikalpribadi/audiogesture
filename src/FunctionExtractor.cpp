/* 
 * File:   FunctionExtractor.cpp
 * Author: haikalpribadi
 * 
 * Created on 02 March 2014, 20:46
 */

#include "FunctionExtractor.h"

FunctionExtractor::FunctionExtractor() {
    ros::NodeHandle node_p("~");
    if(node_p.getParam("feature", feature)) {
        ROS_INFO("FunctionExtractor using feature: %s", feature.c_str());
        subscriber = node.subscribe(feature, 1000, 
                                         &FunctionExtractor::extractorCallback, this);
    }
    else {
        ROS_ERROR("Please set the feature parameter for FunctionExtractor");
        ros::requestShutdown();
    }
}

void FunctionExtractor::extractorCallback(const std_msgs::Float32::ConstPtr& msg) {
    ROS_INFO("%f received", msg->data);
}


int main(int argc, char ** argv) {
    ros::init(argc, argv, "FunctionExtractor");
    
    FunctionExtractor func;
    
    ros::spin();
}