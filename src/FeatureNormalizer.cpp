/* 
 * File:   FeatureNormalizer.cpp
 * Author: haikalpribadi
 * 
 * Created on 04 March 2014, 17:57
 */

#include "FeatureNormalizer.h"

float FeatureNormalizer::fv_max = 0.0;
float FeatureNormalizer::fv_min = 0.0;
ofstream FeatureNormalizer::file;
string FeatureNormalizer::filename = "";

FeatureNormalizer::FeatureNormalizer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("FeatureNormalizer using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for FeatureNormalizer");
        ros::requestShutdown();
    }
    
    featureVector_sub = node.subscribe("feature_vector", 1000,
            &FeatureNormalizer::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &FeatureNormalizer::extractorStatusCallback, this);

}

void FeatureNormalizer::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    vector<float> vector = msg->data;
    featureVectors.push_back(vector);
    
    float current_min = *min_element(vector.begin(), vector.end());
    fv_min = current_min < fv_min ? current_min : fv_min;
    
    float current_max = *max_element(vector.begin(), vector.end());
    fv_max = current_max > fv_max ? current_max : fv_max;
}

void FeatureNormalizer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if (msg->status == "end") {
        normalizeFeatureVectors();
        outputToFile(msg->name);
    }

    ROS_INFO("%s has %s", msg->name.c_str(), msg->status.c_str());
}

void FeatureNormalizer::normalizeFeatureVectors() {
    if(featureVectors.size()<1)
        return;
    
    transform(featureVectors.begin(), featureVectors.end(), featureVectors.begin(), normalize_m);
    /*
    for(int i=0; i<featureVectors.size(); i++) {
        for(int j=0; j<featureVectors[i].size(); j++) {
            float val = featureVectors[i][j];
            featureVectors[i][j] = (val-fv_min)/(fv_max-fv_min);
        }
    }
    */
}

void FeatureNormalizer::outputToFile(string name) {
    if(featureVectors.size()<1)
        return;
    
    filename = music_dir + "/" + name + FV;
    file.open(filename.c_str());
    
    transform(featureVectors.begin(), featureVectors.end(), featureVectors.begin(), output_m);
    
    file.close();
    
    featureVectors.clear();
    fv_min = 0.0;
    fv_max = 0.0;
    
    /*
    string filename = music_dir + "/" + name + FV;
    ofstream file;
    file.open(filename.c_str());

    for(int i=0; i<featureVectors.size(); i++) {
        for(int j=0; j<featureVectors[i].size(); j++) {
            file << featureVectors[i][j];
            if(j < featureVectors[i].size()-1)
                file << ",";
        }
        file << endl;
    }
    
    file << endl;
    file << "Min: " << fv_min << endl;
    file << "Max: " << fv_max << endl;
    
    file.close();
    
    fv_min = 0.0;
    fv_max = 0.0;
    */
}

vector<float> FeatureNormalizer::normalize_m(vector<float> vector) {
    transform(vector.begin(), vector.end(), vector.begin(), normalize_v);
    return vector;
}

float FeatureNormalizer::normalize_v(float val) {
    return (val-fv_min)/(fv_max-fv_min);
}

vector<float> FeatureNormalizer::output_m(vector<float> vector) {
    transform(vector.begin(), vector.end(), vector.begin(), output_v);
    file << endl;
    return vector;
}

float FeatureNormalizer::output_v(float val) {
    file << val << ",";
    return val;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "FeatureNormalizer");

    FeatureNormalizer normalizer;

    ros::spin();
}
