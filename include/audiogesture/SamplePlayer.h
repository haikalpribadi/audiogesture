/* 
 * File:   SamplePlayer.h
 * Author: haikalpribadi
 *
 * Created on 11 March 2014, 01:24
 */

#ifndef SAMPLEPLAYER_H
#define	SAMPLEPLAYER_H

#include <ros/ros.h>

#include "audiogesture/PlayerCommand.h"
#include "audiogesture/TrainerStatus.h"

using namespace std;


class SamplePlayer {
public:
    SamplePlayer();
    
    void playSample();
    
private:
    ros::NodeHandle node;
    ros::Subscriber playerCommand_sub;
    ros::Publisher trainerStatus_pub;
    
    string music_dir;
    string sample;
    string file;
    bool play;
    bool record;
    
    void playerCommandCallback(const audiogesture::PlayerCommand::ConstPtr& msg);
};

#endif	/* SAMPLEPLAYER_H */

