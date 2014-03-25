/* 
 * File:   AudioGestureServer.h
 * Author: haikalpribadi
 *
 * Created on 09 March 2014, 14:26
 */

#ifndef AUDIOGESTURESERVER_H
#define	AUDIOGESTURESERVER_H

#include <iostream>
#include <map>
#include <ros/ros.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/ProcessedOutput.h"
#include "audiogesture/GetFile.h"
#include "audiogesture/GetSamples.h"
#include "audiogesture/TrainerStatus.h"
#include "Sample.h"

using namespace std;

class AudioGestureServer {
public:
    AudioGestureServer();

private:
    ros::NodeHandle node;
    ros::ServiceServer deleteLastGestureFile_srv;
    ros::ServiceServer deleteSample_srv;
    ros::ServiceServer getLastGestureFile_srv;
    ros::ServiceServer getSampleFile_srv;
    ros::ServiceServer getSamples_srv;
    ros::Subscriber extractorStatus_sub;
    ros::Subscriber processedOutput_sub;
    ros::Subscriber trainerStatus_sub;
    
    string music_dir;
    map<string, Sample> samples;
    
    bool deleteLastGestureFile(audiogesture::GetFile::Request &req,
                                  audiogesture::GetFile::Response &res);
    bool deleteSample(audiogesture::GetFile::Request &req,
                        audiogesture::GetFile::Response &res);
    bool getLastGestureFile(audiogesture::GetFile::Request &req,
                               audiogesture::GetFile::Response &res);
    bool getSampleFile(audiogesture::GetFile::Request &req,
                        audiogesture::GetFile::Response &res);
    bool getSamples(audiogesture::GetSamples::Request &req,
                     audiogesture::GetSamples::Response &res);
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    void processedOutputCallback(const audiogesture::ProcessedOutput::ConstPtr& msg);
    void trainerStatusCallback(const audiogesture::TrainerStatus::ConstPtr& msg);
    
    void print();
};

#endif	/* AUDIOGESTURESERVER_H */

