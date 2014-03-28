/*
 * File:   AudioGestureServer.cpp
* Author: haikalpribadi
*
* Created on 09 March 2014, 14:26
*/

#include "AudioGestureServer.h"
#include "audiogesture/PCAExtractor.h"

AudioGestureServer::AudioGestureServer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("AudioGestureServer using music_dir: %s", music_dir.c_str());
        chdir(music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_dir parameter for AudioGestureServer");
        ros::requestShutdown();
    }
    
    deleteLastGestureFile_srv = node.advertiseService("delete_last_gesture_file", &AudioGestureServer::deleteLastGestureFile, this);
    deleteSample_srv = node.advertiseService("delete_sample", &AudioGestureServer::deleteSample, this);
    getLastGestureFile_srv = node.advertiseService("get_last_gesture_file", &AudioGestureServer::getLastGestureFile, this);
    getSampleFile_srv = node.advertiseService("get_sample_file", &AudioGestureServer::getSampleFile, this);
    getSamples_srv = node.advertiseService("get_samples", &AudioGestureServer::getSamples, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
                                         &AudioGestureServer::extractorStatusCallback, this);
    processedOutput_sub = node.subscribe("processed_output", 1000,
                                         &AudioGestureServer::processedOutputCallback, this);
    trainerStatus_sub = node.subscribe("trainer_status", 1000,
                                       &AudioGestureServer::trainerStatusCallback, this);
}

void PCAExtractor::setupNode() {
    
}

bool AudioGestureServer::deleteLastGestureFile(audiogesture::GetFile::Request& req, audiogesture::GetFile::Response& res) {
    string name = req.name;
    
    if(samples.find(name) == samples.end())
        return false;
    
    if(samples.find(name)->second.gestureFiles.empty())
        return false;
    
    string file = samples.find(name)->second.gestureFiles.back();
    string pathfile = music_dir + "/" + file;
    remove(pathfile.c_str());
    
    samples.find(name)->second.gestureFiles.pop_back();
    return true;
}

bool AudioGestureServer::deleteSample(audiogesture::GetFile::Request& req, audiogesture::GetFile::Response& res) {
    string name = req.name;
    
    if(samples.find(name) == samples.end())
        return false;
    
    Sample sample = samples.at(name);
    remove((music_dir + "/" + sample.collectionFile).c_str());
    remove((music_dir + "/" + sample.featureFile).c_str());
    remove((music_dir + "/" + sample.featureNormFile).c_str());
    remove((music_dir + "/" + sample.sampleFile).c_str());
    for(int i=0; i<sample.gestureFiles.size(); i++) {
        remove((music_dir + "/" + sample.gestureFiles[i]).c_str());
    }
    
    samples.erase(name);
    return true;
}


bool AudioGestureServer::getLastGestureFile(audiogesture::GetFile::Request& req, audiogesture::GetFile::Response& res) {
    string name = req.name;
    
    if(samples.find(name) == samples.end())
        return false;
    
    if(samples.find(name)->second.gestureFiles.empty())
        return false;
    
    res.file = samples.find(name)->second.gestureFiles.back();
    return true;
}

bool AudioGestureServer::getSampleFile(audiogesture::GetFile::Request& req, audiogesture::GetFile::Response& res) {
    string name = req.name;
    
    if(samples.find(name) == samples.end())
        return false;
    
    res.file = samples.find(name)->second.sampleFile;
    
    return true;
}

bool AudioGestureServer::getSamples(audiogesture::GetSamples::Request& req, audiogesture::GetSamples::Response& res) {
    map<string, Sample>::iterator it;
    for(it = samples.begin(); it != samples.end(); ++it) {
        res.samples.push_back(it->first);
    }
    return true;
}


void AudioGestureServer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    
}

void AudioGestureServer::processedOutputCallback(const audiogesture::ProcessedOutput::ConstPtr& msg) {
    string name = msg->name;
    string type = msg->type;
    string file = msg->file;
    
    if(samples.find(name) == samples.end()) {
        Sample newsample(name);
        samples.insert(pair<string, Sample>(name, newsample));
    }
    
    Sample& sample = samples.find(name)->second;
    
    if(type == "sample")
        sample.setSample(file);
    else if(type == "collection")
        sample.setCollection(file);
    else if(type == "feature")
        sample.setFeature(file);
    else if(type == "featurenorm")
        sample.setFeatureNorm(file);
    else if(type == "gesture") {
        sample.addGestureFile(file);
    }
}

void AudioGestureServer::trainerStatusCallback(const audiogesture::TrainerStatus::ConstPtr& msg) {
    string name = msg->name;
    string status = msg->status;
    
    if(status == "start") {
        
    } else if(status == "end") {
        print();
    }
}

void AudioGestureServer::print() {
    map<string,Sample>::iterator it;
    cout << "===========================================" << endl;
    cout << "Samples contains: " << endl;
    for (it=samples.begin(); it!=samples.end(); ++it) {
        Sample& sample = it->second;
        cout << it->first << " => " << sample.name << " ";
        cout << sample.collectionFile << " ";
        cout << sample.featureFile << " ";
        cout << sample.featureNormFile << " ";
        cout << sample.sampleFile << " " << endl;
        for(int i=0; i<it->second.gestureFiles.size(); i++) {
            cout << it->second.gestureFiles[i] << " ";
        }
        cout << endl;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "AudioGestureServer");
    
    AudioGestureServer server;
    
    ros::spin();
    
    return 0;
}