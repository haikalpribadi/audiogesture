/* 
 * File:   SampleListener.h
 * Author: haikalpribadi
 *
 * Created on 13 February 2014, 11:36
 */

#ifndef SAMPLELISTENER_H
#define	SAMPLELISTENER_H

#include <algorithm>
#include <cstdlib>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sys/inotify.h>

#include "audiogesture/MusicExtractor.h"


#define EVENT_SIZE      ( sizeof ( struct inotify_event ))
#define BUF_LEN         ( 1024 * ( EVENT_SIZE + NAME_MAX + 1 ))
#define WATCH_FLAGS     ( IN_CREATE | IN_MOVED_TO )
#define MF      ".mf"
#define ARFF    ".arff"

using namespace std;

static bool run = true;

class SampleListener {
public:
    SampleListener();
    SampleListener(const SampleListener& orig);
    virtual ~SampleListener();
    
    void monitorDirectory();
    
private:
    bool run;
    string music_dir;
    
    ros::NodeHandle node;
    ros::Publisher featureExtractor_pub;
    
    void generateCollectionFile(string sampleName);
    void publishToFeatureExtraction(string filename);
    bool hasFormat(string file, string format);
};

void signalCallback(int signal);

#endif	/* SAMPLELISTENER_H */

