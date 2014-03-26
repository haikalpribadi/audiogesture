/*
* File: AudioInput.h
* Author: haikalpribadi
*
* Created on 25 March 2014, 12:09
*/

#ifndef AUDIOINPUT_H
#define AUDIOINPUT_H

#include <ros/ros.h>
#include <signal.h>
#include <sstream>

#include "audiogesture/GetFile.h"

using namespace std;

static bool run = true;

class AudioInput {
public:
    AudioInput();
    void process();
    string music_dir;
    string command;
    string output;
    int duration;
    int counter;
    int log;
    
private:
    ros::NodeHandle node;
    ros::ServiceClient deleteSample_cl;
    ros::ServiceClient getSampleFile_cl;
    

};

void signalCallback(int signal);

#endif /* AUDIOINPUT_H */


