/* 
 * File:   OutputRecorder.cpp
 * Author: haikalpribadi
 * 
 * Created on 09 June 2014, 05:11
 */

#include "OutputRecorder.h"

OutputRecorder::OutputRecorder() {
    if (node.getParam("pca_dir", pca_dir)) {
        chdir(pca_dir.c_str());
        if(!(stat("output", &sb) == 0 && S_ISDIR(sb.st_mode))) {
            mkdir("output", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }
        output_dir = pca_dir + "/output";
        chdir(output_dir.c_str());
    } else {
        ROS_ERROR("Please set the pca_dir parameter for PCARegression");
        ros::requestShutdown();
    }
    
    recordCounter = 0;
    record = false;
    
    status_sub = node.subscribe("record_status", 1000,
            &OutputRecorder::statusCallback, this);
    output_sub = node.subscribe("output_record", 1000,
            &OutputRecorder::recordCallback, this);
    
}

void OutputRecorder::statusCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "start") {
        record = true;
        recordCounter++;
        
        stringstream path1;
        path1 << "feature_output_" << recordCounter;
        featureFile.open(path1.str().c_str());
        
        stringstream path2;
        path2 << "gesture_output_" << recordCounter;
        gestureFile.open(path2.str().c_str());
        
    } else if(msg->data == "stop") {
        record = false;
        featureFile.close();
        gestureFile.close();
    }
}

void OutputRecorder::recordCallback(const audiogesture::OutputRecord::ConstPtr& msg) {
    if(!record)
        return;
    
    for(int i=0; i<msg->feature_data.size(); i++) {
        featureFile << msg->feature_data[i] << ",";
    }
    featureFile << endl;
    
    for(int i=0; i<msg->gesture_data.size(); i++) {
        gestureFile << msg->gesture_data[i] << ",";
    }
    gestureFile << endl;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "OutputRecorder");
    
    OutputRecorder out;
    
    ros::spin();
    
    return 0;
}