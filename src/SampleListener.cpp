/* 
 * File:   SampleListener.cpp
 * Author: haikalpribadi
 * 
 * Created on 13 February 2014, 11:36
 */

#include "SampleListener.h"

SampleListener::SampleListener() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("MusicSampleListener using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_dir (file) parameter for extractor");
        ros::requestShutdown();
    }
    
    if (!node.getParam("bextract_args", args)) {
        args = "";
    }
    
    if (!node.getParam("input_mode", input)) {
        input = FOLDER;
    }
    ROS_INFO("MusicSampleListener input set to: %s", input.c_str());
    
    featureExtractor_pub = node.advertise<std_msgs::String>("music_extractor_buffer", 1000);
    collectionGenerator_pub = node.advertise<std_msgs::String>("collection_generator", 1000);
    processedOutput_pub = node.advertise<audiogesture::ProcessedOutput>("processed_output", 1000);
}

void SampleListener::monitorDirectory() {
    int descriptor = inotify_init();
    int watched_dir;
    char buffer[BUF_LEN];

    if (descriptor >= 0) {
        watched_dir = inotify_add_watch(descriptor, music_dir.c_str(), WATCH_FLAGS);
    } else {
        ROS_ERROR("Unable to listen to music directory");
        ros::requestShutdown();
    }

    signal(SIGINT, signalCallback);

    int length, i = 0;
    while (run) {
        while (run && (length = read(descriptor, buffer, BUF_LEN)) >= 0) {
            i = 0;
            while (i < length) {
                struct inotify_event *event = (struct inotify_event *) &buffer[i];
                i += EVENT_SIZE + event->len;
                if (event->len && !(event->mask & IN_ISDIR)) {
                    string name = event->name;
                    ROS_INFO("%s generated", name.c_str());
                    if (name == "bextract_single.mf" ||
                        name == "weka.arff")
                        continue;
                    
                    if (hasFormat(event->name, ARFF)) {

                    } else if (hasFormat(name, MF)) {
                        publishToFeatureExtraction(name);
                    } else if (hasFormat(name, FV)) {

                    } else if (hasFormat(name, HTML)) {

                    } else if (name.find(' ') != -1) {
                        renameFile(name);
                    } else if (input==FOLDER) {
                        publishToCollectionGenerator(name);
                    }
                    
                    publishToProcessedOutput(name);
                }
            }
        }
    }

    inotify_rm_watch(descriptor, watched_dir);
    close(descriptor);
}

void SampleListener::publishToCollectionGenerator(string filename) {
    std_msgs::String msg;
    msg.data = filename;

    collectionGenerator_pub.publish(msg);
    ROS_INFO("Collection Generator called for: %s", msg.data.c_str());
    return;
}

void SampleListener::publishToFeatureExtraction(string filename) {
    string outname = filename.substr(0, filename.find(".")) + ARFF;
    std_msgs::String msg;
    msg.data = filename + " -w " + outname;
    if (args != "")
        msg.data = msg.data + " " + args;

    featureExtractor_pub.publish(msg);
    ROS_INFO("Music Feature Extraction called for: %s", msg.data.c_str());
}

void SampleListener::publishToProcessedOutput(string filename) {
    audiogesture::ProcessedOutput msg;
    msg.name = filename.substr(0, filename.find("."));
    msg.file = filename;
    
    if (hasFormat(filename, ARFF)) {
        msg.type = "feature";
    } else if (hasFormat(filename, MF)) {
        msg.type = "collection";
    } else if (hasFormat(filename, FV)) {
        msg.type = "featurenorm";
    } else if (hasFormat(filename, HTML)) {
        
    } else {
        msg.type = "sample";
    }
    
    processedOutput_pub.publish(msg);
}

bool SampleListener::hasFormat(string filename, string format) {
    if (filename.length() >= format.length()) {
        return (0 == filename.compare(filename.length() - format.length(), format.length(), format));
    }

    return false;
}

void SampleListener::renameFile(string filename) {
    string newsample = filename;
    replace(newsample.begin(), newsample.end(), ' ', '_');

    string pathsample = music_dir + "/" + filename;
    string pathnewsample = music_dir + "/" + newsample;

    int result = rename(pathsample.c_str(), pathnewsample.c_str());

    if (result != 0) {
        ROS_ERROR("Sample %s not successfully renamed", filename.c_str());
    }
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
