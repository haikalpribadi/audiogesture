/* 
 * File:   AudioGestureTrainer.cpp
 * Author: haikalpribadi
 * 
 * Created on 06 March 2014, 16:21
 */

#include "AudioGestureTrainer.h"

AudioGestureTrainer::AudioGestureTrainer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("AudioGestureTrainer using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for AudioGestureTrainer");
        ros::requestShutdown();
    }
    
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
                                         &AudioGestureTrainer::extractorStatusCallback, this);
    
    
    //TODO : this is temporary testing publisher
    trainerStatus_pub = node.advertise<audiogesture::TrainerStatus>("trainer_status", 1000);
    
}

void AudioGestureTrainer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AudioGestureTrainer");
    
    AudioGestureTrainer trainer;
    
    audiogesture::TrainerStatus msg;
    msg.name = "water1";
    msg.status = "start";
    usleep(1000000);
    trainer.trainerStatus_pub.publish(msg);
    usleep(3000000);
    msg.status = "end";
    trainer.trainerStatus_pub.publish(msg);
    
    //ros::spin();
}