/* 
 * File:   SamplePlayer.cpp
 * Author: haikalpribadi
 * 
 * Created on 11 March 2014, 01:24
 */

#include "SamplePlayer.h"

SamplePlayer::SamplePlayer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("SamplePlayer using music_dir: %s", music_dir.c_str());
        chdir(music_dir.c_str());
    } else {
        ROS_ERROR("SamplePlayer set the music_directory (file) parameter for AudioGestureServer");
        ros::requestShutdown();
    }
    
    play = false;
    record = false;
    playerCommand_sub = node.subscribe("player_command", 1000,
                                       &SamplePlayer::playerCommandCallback, this);
    trainerStatus_pub = node.advertise<audiogesture::TrainerStatus>("trainer_status", 1000);
    
}

void SamplePlayer::playSample() {
    if(play) {
        if(record) {
            audiogesture::TrainerStatus msg;
            msg.name = sample;
            msg.status = "start";
            trainerStatus_pub.publish(msg);
        }

        string cmd = "mplayer " + file;
        system(cmd.c_str());
        
        if(record) {
            audiogesture::TrainerStatus msg;
            msg.name = sample;
            msg.status = "end";
            trainerStatus_pub.publish(msg);
        }
        
        play = false;
        usleep(2000000);
    }
}

void SamplePlayer::playerCommandCallback(const audiogesture::PlayerCommand::ConstPtr& msg) {
    sample = msg->name;
    file = msg->file;
    string command = msg->command;
    
    ROS_INFO("%s %s with file %s", command.c_str(), sample.c_str(), file.c_str());
    
    if(command=="play") {
        play = true;
        record = false;
    } else if(command=="record") {
        play = true;
        record = true;
    } else if(command=="stop") {
        play = false;
        record = false;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "SamplePlayer");
    
    SamplePlayer player;
    ros::Rate rate(100);
    
    while(ros::ok()) {
        ros::spinOnce();
        player.playSample();
        rate.sleep();
    }
}

