/* 
 * File:   GestureReceiver.h
 * Author: haikalpribadi
 *
 * Created on 05 March 2014, 19:56
 */

#ifndef GESTURERECEIVER_H
#define	GESTURERECEIVER_H

#include <cstdlib>
#include <fstream>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "audiogesture/TrainerStatus.h"
#include "audiogesture/ProcessedOutput.h"

#define GV  ".gv"

using namespace std;


class GestureReceiver {
public:
    GestureReceiver();
    
    struct stat sb;
    
private:
    ros::NodeHandle node;
    ros::Publisher processedOutput_pub;
    ros::Subscriber gestureVector_sub;
    //ros::Subscriber gestureMessage_sub;
    ros::Subscriber recordSwitch_sub;
    ros::Subscriber trainerStatus_sub;
    
    string music_dir;
    string samplename;
    string filename;
    bool output;
    int id;
    
    ofstream file;
    
    //void gestureMessageCallback(const std_msgs::String::ConstPtr& msg);
    void gestureVectorCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void trainerStatusCallback(const audiogesture::TrainerStatus::ConstPtr& msg);
    
    void outputToFile(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void publishToProcessedOutput();
    void startOutputTofile();
    void stopOutputToFile();
};

#endif	/* GESTURERECEIVER_H */

