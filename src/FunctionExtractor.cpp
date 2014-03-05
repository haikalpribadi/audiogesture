/* 
 * File:   FunctionExtractor.cpp
 * Author: haikalpribadi
 * 
 * Created on 02 March 2014, 20:46
 */

#include "FunctionExtractor.h"

FunctionExtractor::FunctionExtractor() {
    extractor_sub = node.subscribe("feature_vector", 1000,
            &FunctionExtractor::extractorCallback, this);

    /* 
     * This block allows subscribe each feature on individual channel
     * 
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
     */
    ROS_INFO("FunctionExtractor started listening to feature vectors");
}

void FunctionExtractor::extractorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    /*
    stringstream ss;
    ss << "[";
    for(int i=0; i<msg->data.size(); i++) {
        ss << msg->data[i];
        if(i<msg->data.size()-1)
            ss << ", ";
        else
            ss << "]";
    }
        
    ROS_INFO("received: %s", ss.str().c_str());
     */
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "FunctionExtractor");

    FunctionExtractor func;

    ros::spin();
}