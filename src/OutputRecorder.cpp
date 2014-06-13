/* 
 * File:   OutputRecorder.cpp
 * Author: haikalpribadi
 * 
 * Created on 09 June 2014, 05:11
 */

#include "OutputRecorder.h"
#include "audiogesture/MagnitudeRecord.h"

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
    magnitude_sub = node.subscribe("magnitude_record", 1000,
            &OutputRecorder::magnitudeCallback, this);
    
}

void OutputRecorder::statusCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "start") {
        record = true;
        recordCounter++;
        
        stringstream path1;
        path1 << "output_feature_" << recordCounter;
        featureFile.open(path1.str().c_str());
        
        stringstream path2;
        path2 << "output_gesture_" << recordCounter;
        gestureFile.open(path2.str().c_str());
        
        
    } else if(msg->data == "stop") {
        record = false;
        featureFile.close();
        gestureFile.close();
        
        if(featureMagnitudes.size()>0) {
            stringstream path3;
            path3 << "magnitude_feature_" << recordCounter;
            featureMagFile.open(path3.str().c_str());
            for(int j=0; j<featureMagnitudes[0].size(); j++) {
                for(int i=0; i<featureMagnitudes.size(); i++) {
                    featureMagFile << fixed << setprecision(5) << featureMagnitudes[i][j] << ",";
                }
                featureMagFile << endl;
            }
            featureMagnitudes.clear();
            
            stringstream path4;
            path4 << "magnitude_gesture_" << recordCounter;
            gestureMagFile.open(path4.str().c_str());
            for(int j=0; j<gestureMagnitudes[0].size(); j++) {
                for(int i=0; i<gestureMagnitudes.size(); i++) {
                    gestureMagFile << fixed << setprecision(5) << gestureMagnitudes[i][j] << ",";
                }
                gestureMagFile << endl;
            }
            gestureMagnitudes.clear();
        }
    }
}

void OutputRecorder::magnitudeCallback(const audiogesture::MagnitudeRecord::ConstPtr& msg) {
    if(!record)
        return;
    
    featureMagnitudes.push_back(msg->feature_data);
    gestureMagnitudes.push_back(msg->gesture_data);
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