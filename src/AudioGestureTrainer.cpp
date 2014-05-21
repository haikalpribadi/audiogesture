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
        ROS_ERROR("Please set the music_dir parameter for AudioGestureTrainer");
        ros::requestShutdown();
    }
    
    kinectCalibrate_pub = node.advertise<std_msgs::Empty>("kinect_calibrate", 1000);
    playerCommand_pub = node.advertise<audiogesture::PlayerCommand>("player_command", 1000);
    trainerStatus_pub = node.advertise<audiogesture::TrainerStatus>("trainer_status", 1000);
    trainerLogStatus_pub = node.advertise<audiogesture::TrainerLogStatus>("trainer_log_status", 1000);
    deleteLastGestureFile_cl = node.serviceClient<audiogesture::GetFile>("delete_last_gesture_file");
    getLastGestureFile_cl = node.serviceClient<audiogesture::GetFile>("get_last_gesture_file");
    getSampleFile_cl = node.serviceClient<audiogesture::GetFile>("get_sample_file");
    getSamples_cl = node.serviceClient<audiogesture::GetSamples>("get_samples");
}

void AudioGestureTrainer::run() {
    string command = "";
    bool next = true;
    int id;
    
    while(ros::ok() && command!="q" && next) {
        audiogesture::GetSamples srv;
        getSamples_cl.call(srv);
        
        vector<string> samples(srv.response.samples);
        sort(samples.begin(), samples.end(), comparenat);
        printSamples(samples);
        
        cout << "Enter sample ID to start training (q to quit, c to calibrate): ";
        cin >> command;
        
        if(command == "c") {
            std_msgs::Empty msg;
            kinectCalibrate_pub.publish(msg);
        }
        
        if(command == "q" || !isNumber(command))
            continue;
        
        stringstream(command) >> id;
        if(id>samples.size())
            continue;
        
        string sample = samples[id-1];
        next = trainSample(sample);
        
    }
}

bool AudioGestureTrainer::trainSample(string sample) {
    audiogesture::GetFile srv;
    srv.request.name = sample;
    getSampleFile_cl.call(srv);
    string file = srv.response.file;
    string command;
    
    cout << "---------------------------------------------";
    cout << "---------------------------------------------" << endl;
    cout << "Training sample: " << sample << endl;
    
    cout << "(a) play   \t";
    cout << "(s) record \t";
    cout << "(d) stop   \t";
    cout << "(f) delete \t";
    cout << "(g) menu   \t";
    cout << "(q) quit";
    cout << endl;

    while(ros::ok() && command!="q" && command!="g") {
        if(command=="a") {
            publishToPlay(sample, file);
        } else if (command=="s") {
            publishToRecord(sample, file);
        } else if (command=="d") {
            publishToStop(sample, file);
        } else if (command=="f") {
            deleteLastGestureFile(sample);
        } else if (command=="g") {
            publishToStop(sample, file);
        }
        cout << "> ";
        cin >> command;
    }
    
    if(command == "q")
        return false;
    
    return true;
}

void AudioGestureTrainer::deleteLastGestureFile(string sample) {
    audiogesture::GetFile srv;
    srv.request.name = sample;
    deleteLastGestureFile_cl.call(srv);
}

void AudioGestureTrainer::publishToPlay(string sample, string file) {
    audiogesture::PlayerCommand msg;
    msg.name = sample;
    msg.file = file;
    msg.command = "play";
    playerCommand_pub.publish(msg);
}

void AudioGestureTrainer::publishToRecord(string sample, string file) {
    audiogesture::PlayerCommand msg;
    msg.name = sample;
    msg.file = file;
    msg.command = "record";
    playerCommand_pub.publish(msg);
    
    audiogesture::TrainerLogStatus msg2;
    msg2.name = sample;
    msg2.status = "start";
    trainerLogStatus_pub.publish(msg2);
}

void AudioGestureTrainer::publishToStop(string sample, string file) {
    audiogesture::PlayerCommand msg;
    msg.name = sample;
    msg.file = file;
    msg.command = "stop";
    playerCommand_pub.publish(msg);
    
    audiogesture::TrainerLogStatus msg2;
    msg2.name = sample;
    msg2.status = "end";
    trainerLogStatus_pub.publish(msg2);
}

void AudioGestureTrainer::printSamples(const vector<string>& samples) {
    int size = samples.size();
    int j=0;
    int columns = 4; //min(4,(int)ceil(size/2.0));
    int rows = ceil((float)size/columns);
    cout << endl << "=============================================";
    cout << "=============================================" << endl;
    for(int i=0; i<(columns*rows); i++) {
        int x = (((i%columns)*rows)%size)+(floor((float)i/columns));
        if(x>=size || ((i%columns)*rows)>=size) {
            cout << endl;
            j = 0;
            continue;
        }
        cout << (x+1) << " - " << samples[x] << "\t\t";
        j++;
        if(j==columns) {
            cout << endl;
            j = 0;
        }
    }
    cout << endl;
}

bool AudioGestureTrainer::isNumber(const std::string& s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && std::isdigit(*it)) ++it;
    return !s.empty() && it == s.end();
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
    
    return 0;
}