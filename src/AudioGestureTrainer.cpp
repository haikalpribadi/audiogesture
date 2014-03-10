/* 
 * File:   AudioGestureTrainer.cpp
 * Author: haikalpribadi
 * 
 * Created on 06 March 2014, 16:21
 */

#include <cmath>

#include "AudioGestureTrainer.h"

AudioGestureTrainer::AudioGestureTrainer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("AudioGestureTrainer using music_dir: %s", music_dir.c_str());
        chdir(music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for AudioGestureTrainer");
        ros::requestShutdown();
    }
    
    trainerStatus_pub = node.advertise<audiogesture::TrainerStatus>("trainer_status", 1000);
    getSampleFile_cl = node.serviceClient<audiogesture::GetSampleFile>("get_sample_file");
    getSamples_cl = node.serviceClient<audiogesture::GetSamples>("get_samples");
}

void AudioGestureTrainer::run() {
    string command = "";
    
    while(ros::ok() && command!="q") {
        audiogesture::GetSamples srv;
        getSamples_cl.call(srv);
        
        vector<string> samples(srv.response.samples);
        sort(samples.begin(), samples.end(), comparenat);
        printSamples(samples);
        
        cin >> command;
    }
}

void AudioGestureTrainer::printSamples(const vector<string>& samples) {
    int size = samples.size();
    int j=0;
    int column = 4;
    int row = ceil((float)size/column);
    cout << endl << "=============================================";
    cout << "=============================================" << endl;
    for(int i=0; i<(column*row); i++) {
        int x = (((i%column)*row)%size)+(floor((float)i/column));
        if(x>=size) {
            cout << endl;
            j = 0;
            continue;
        }
        cout << (x+1) << " - " << samples[x] << "\t\t";
        j++;
        if(j==4) {
            cout << endl;
            j = 0;
        }
    }
    cout << endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AudioGestureTrainer");
    
    AudioGestureTrainer trainer;
    trainer.run();
    
    /*
    audiogesture::GetSampleFile srv;
    srv.request.name = "water1";
    trainer.getSampleFile_cl.call(srv);
    string file = srv.response.file;
    string cmd = "mplayer " + file;
    
    audiogesture::TrainerStatus msg;
    msg.name = "water1";
    msg.status = "start";
    usleep(1000000);
    trainer.trainerStatus_pub.publish(msg);
    system(cmd.c_str());
    msg.status = "end";
    trainer.trainerStatus_pub.publish(msg);
     */
    
    //ros::spin();
}