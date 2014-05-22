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
#include <dirent.h>
#include <ros/ros.h>
#include <signal.h>
#include <stdio.h>
#include <std_msgs/String.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "audiogesture/ProcessedOutput.h"
#include "CompareNatural.h"

#define EVENT_SIZE      ( sizeof ( struct inotify_event ))
#define BUF_LEN         ( 1024 * ( EVENT_SIZE + NAME_MAX + 1 ))
#define WATCH_FLAGS     ( IN_CREATE | IN_MOVED_TO )

#define ARFF    ".arff"
#define FV      ".fv"
#define HTML    ".html"
#define MF      ".mf"
#define WAV     ".wav"
#define LIVE    "live"
#define FOLDER  "folder"

using namespace std;

struct stat sb;
static bool run = true;

class SampleListener {
public:
    SampleListener();
    void monitorDirectory();
    void cleanDirectory();
    
private:
    bool run;
    string input;
    string music_dir;
    string args;
    
    ros::NodeHandle node;
    ros::Publisher featureExtractor_pub;
    ros::Publisher collectionGenerator_pub;
    ros::Publisher processedOutput_pub;
    
    void publishToCollectionGenerator(string filename);
    void publishToFeatureExtraction(string fiename);
    void publishToProcessedOutput(string filename);
    bool hasFormat(string filename, string format);
    void renameFile(string filename);
    
    vector<string> readDirectory(const string& path);
};

void signalCallback(int signal);

#endif	/* SAMPLELISTENER_H */

