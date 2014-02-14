/* 
 * File:   SampleListener.cpp
 * Author: haikalpribadi
 * 
 * Created on 13 February 2014, 11:36
 */

#include "SampleListener.h"

SampleListener::SampleListener() {
    if(node.getParam("music_dir", music_dir)) {
        ROS_INFO("MusicSampleListener using music_dir: %s", music_dir.c_str());
    }
    else {
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }
    
    featureExtractor_pub = node.advertise<std_msgs::String>("music_extractor", 1000);
    collectionGenerator_pub = node.advertise<std_msgs::String>("collection_generator", 1000);
}

void SampleListener::monitorDirectory() {
    int i = 0;
    int descriptor = inotify_init();
    int watched_dir;
    char buffer[BUF_LEN];
    
    if(descriptor >= 0 ) {
        watched_dir = inotify_add_watch(descriptor, music_dir.c_str(), WATCH_FLAGS);
    }
    else {
        ROS_ERROR("Unable to listen to music directory");
        ros::requestShutdown();
    }
    
    signal(SIGINT, signalCallback);
    
    while(run) {
        while(run && read(descriptor, buffer, BUF_LEN)>0) {
            struct inotify_event *event = (struct inotify_event *) &buffer[i];
            if(event->len && !(event->mask & IN_ISDIR))
            {
                string name = event->name;
                ROS_INFO("%s generated", name.c_str());
                if(hasFormat(event->name, ARFF))                {
                    
                }
                else if(hasFormat(name, MF)) {
                    if(name != "bextract_single.mf")
                        publishToFeatureExtraction(name);
                }
                else{
                    publishToCollectionGenerator(name);
                }
            }
        }
    }
    
    inotify_rm_watch(descriptor, watched_dir);
    close(descriptor);
}

void SampleListener::publishToCollectionGenerator(string sample)
{
    std_msgs::String msg;
    msg.data = sample;
    
    collectionGenerator_pub.publish(msg);
    ROS_INFO("Collection Generator called for: %s", msg.data.c_str());
    return;
}

void SampleListener::publishToFeatureExtraction(string filename) {
    string outname = filename.substr(0, filename.find(".")) + ARFF;
    
    string args = "";
    std_msgs::String msg;
    stringstream ss;
    ss << filename << " -w " << outname;
    if(node.getParam("bextract_args", args))
    {
        ss << " " << args;
    }
    
    msg.data = ss.str();
    featureExtractor_pub.publish(msg);
    ROS_INFO("Music Feature Extraction called for: %s", msg.data.c_str());
}

bool SampleListener::hasFormat (string file, string format) {
    if (file.length() >= format.length())
    {
        return (0 == file.compare (file.length() - format.length(), format.length(), format));
    }
    
    return false;
}

void signalCallback(int signal) {
    run = false;
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "MusicSampleListener");
    
    SampleListener listener;
    listener.monitorDirectory();
    
    return 0;
}
