/* 
 * File:   MusicExtractorBuffer.cpp
 * Author: haikalpribadi
 * 
 * Created on 05 March 2014, 12:01
 */

#include "MusicExtractorBuffer.h"

MusicExtractorBuffer::MusicExtractorBuffer() {
    hold = false;
    buffer_pub = node.advertise<std_msgs::String>("music_extractor", 1000);
    buffer_sub = node.subscribe("music_extractor_buffer", 1000,
                                &MusicExtractorBuffer::bufferCallback, this);
    
    ROS_INFO("MusicFeatureExtractorBuffer has started listening to the command buffer");
    
}

void MusicExtractorBuffer::bufferCallback(const std_msgs::String::ConstPtr& msg) {
    messages.push(*msg);
}

void MusicExtractorBuffer::statusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if(msg->status == "start")
        hold = true;
    else if(msg->status == "end")
        hold = false;
}

void MusicExtractorBuffer::processBuffer() {
    if(messages.size()>0 && !hold) {
        buffer_pub.publish(messages.front());
        messages.pop();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "MusicFeatureExtractorBuffer");
    MusicExtractorBuffer buffer;
    ros::Rate rate(1000000);
    
    
    while(ros::ok()) {
        ros::spinOnce();
        buffer.processBuffer();
        rate.sleep();
    }
    
    return 0;
}

