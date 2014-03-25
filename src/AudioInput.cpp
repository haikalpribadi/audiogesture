/* 
 * File:   AudioInput.cpp
 * Author: haikalpribadi
 * 
 * Created on 25 March 2014, 12:09
 */

#include "AudioInput.h"

AudioInput::AudioInput() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("AudioInput using music_dir: %s", music_dir.c_str());
        chdir(music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for AudioInput");
        ros::requestShutdown();
    }

    if (node.getParam("sample_duration", duration)) {
        ROS_INFO("AudioInput is sampling at duration: %d second(s)", duration);
    } else {
        ROS_ERROR("Please set sample_duration for AudioInput");
        ros::requestShutdown();
    }
    
    if (node.getParam("sample_log", log)) {
        ROS_INFO("AudioInput is logging samples up to: %d", log);
    } else {
        ROS_ERROR("Please set sample_log for AudioInput");
        ros::requestShutdown();
    }
    
    deleteSample_cl = node.serviceClient<audiogesture::GetFile>("delete_sample");
    getSampleFile_cl = node.serviceClient<audiogesture::GetFile>("get_sample_file");
    
    counter = 0;
}

void AudioInput::process() {
    stringstream ss;
    ss << "in" << counter++;
    counter = counter % log;
    output = ss.str();
    audiogesture::GetFile srv;
    srv.request.name = output;
    if(getSampleFile_cl.call(srv))
        deleteSample_cl.call(srv);
    
    ss.str("");
    ss.clear();
    ss << "arecord -d " << duration << " -f cd -t wav " << output << ".wav";
    
    command = ss.str();
    int status = system(command.c_str());
    
    if (status!=0) {
        run = false;
    }
}

void signalCallback(int signal) {
    run = false;
    ros::shutdown();
    exit(0);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AudioInput");

    AudioInput input;
    ros::Rate rate(1000);
    signal(SIGKILL, signalCallback);
    
    ROS_INFO("AudioInput waiting, giving time for other nodes to set up ...");
    usleep(5000000);
    while (ros::ok() && run) {
        input.process();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

