/* 
 * File:   GestureReceiver.cpp
 * Author: haikalpribadi
 * 
 * Created on 05 March 2014, 19:56
 */

#include "GestureReceiver.h"

GestureReceiver::GestureReceiver() {
    samplename = "";
    output = false;
    id = 0;
    
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("GestureReceiver using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for GestureReceiver");
        ros::requestShutdown();
    }
    gestureVector_sub = node.subscribe("osc_receive_vector", 1000,
                                       &GestureReceiver::gestureVectorCallback, this);
    gestureMessage_sub = node.subscribe("osc_receive_message", 1000,
                                        &GestureReceiver::gestureMessageCallback, this);
    trainerStatus_sub = node.subscribe("trainer_status", 1000,
                                       &GestureReceiver::trainerStatusCallback, this);
    recordSwitch_sub = node.subscribe("gesture_record", 1000,
                                      &GestureReceiver::recordSwitchCallback, this);
    
    
    
    ROS_INFO("GestureReciever has started listening to gesture control data");
}

void GestureReceiver::gestureMessageCallback(const std_msgs::String::ConstPtr& msg) {
    
}

void GestureReceiver::gestureVectorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    if(output) {
        outputToFile(msg);
    }
}

void GestureReceiver::recordSwitchCallback(const std_msgs::Bool::ConstPtr& msg) {
    record = msg->data;
    if(!record && output) {
        stopOutputToFile();
    }
}

void GestureReceiver::trainerStatusCallback(const audiogesture::TrainerStatus::ConstPtr& msg) {
    samplename = msg->name;
    string status = msg->status;
    
    if(record && status == "start") {
        startOutputTofile();
        
    } else if(status == "end") {
        stopOutputToFile();
    }
}

void GestureReceiver::outputToFile(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    for(int i=0; i<msg->data.size(); i++) {
        file << msg->data[i] << ",";
    }
    file << endl;
}

void GestureReceiver::startOutputTofile() {
    output = true;
    string pathname = music_dir + "/" + samplename;
    if (!(stat(pathname.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))) {
        mkdir(pathname.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    stringstream filename;
    filename << pathname << "/" << samplename << "-" << id++ << GV;
    file.open(filename.str().c_str());
}

void GestureReceiver::stopOutputToFile() {
    output = false;
    file.close();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureReceiver");
    
    GestureReceiver gesture;
    
    ros::spin();
}
