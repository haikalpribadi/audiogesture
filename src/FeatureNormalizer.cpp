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

FeatureNormalizer::FeatureNormalizer() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("FeatureNormalizer using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for FeatureNormalizer");
        ros::requestShutdown();
    }
    if (node.getParam("parameter_dir", parameter_dir)) {
        ROS_INFO("FeatureNormalizer using parameter_dir: %s", parameter_dir.c_str());
    } 
    if (!node.getParam("bextract_args", args)) {
        args = "";
    }
    parameter_file = "ranges" + args + ".txt";
    
    output = true;
    node.getParam("output_to_file", output);
    
    featureVector_pub = node.advertise<audiogesture::FeatureVector>("feature_vector", 1000);
    featureVector_sub = node.subscribe("feature_vector_raw", 1000,
            &FeatureNormalizer::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &FeatureNormalizer::extractorStatusCallback, this);
    
    update = false;
    loadParameters();
}

void FeatureNormalizer::loadParameters() {
    feature_min.clear();
    feature_max.clear();
    
    string path = parameter_dir + "/" + parameter_file;
    string line1, line2, val;
    ifstream file(path.c_str());
    if(file.good() && file.is_open()) {
        if(getline(file,line1) && getline(file,line2)) {
            istringstream stream1(line1);
            while(getline(stream1, val, ',')) {
                feature_min.push_back(atof(val.c_str()));
            }
            istringstream stream2(line2);
            while(getline(stream2, val, ',')) {
                feature_max.push_back(atof(val.c_str()));
            }
            initialize = false;
        }
        file.close();
    }
}

void FeatureNormalizer::storeParameters() {
    string path = parameter_dir + "/" + parameter_file;
    remove(path.c_str());
    
    file.open(path.c_str());
    
    for(int i=0; i<feature_min.size(); i++) {
        file << (floor(feature_min[i] * 100 + 0.5) / 100) << ",";
    }
    file << endl;
    
    for(int i=0; i<feature_max.size(); i++) {
        file << (floor(feature_max[i] * 100 + 0.5) / 100) << ",";
    }
    file << endl;
    
    file.close();
}

void FeatureNormalizer::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    vector<float> vector;
    audiogesture::FeatureVector msg2;
    
    if(initialize) {
        for(int i=0; i<msg->data.size(); i++) {
            feature_min.push_back(0);
            feature_max.push_back(0);
        }
        initialize = false;
    }
    
    for(int i=0; i<msg->data.size(); i++) {
        if(msg->data[i] < feature_min[i]) {
            feature_min[i] = msg->data[i];
            update = true;
        }
        if(msg->data[i] > feature_max[i]) {
            feature_max[i] = msg->data[i];
            update = true;
        }
        
        if(feature_max[i]-feature_min[i] == 0)
            vector.push_back(0);
        else
            vector.push_back((msg->data[i]-feature_min[i])/(feature_max[i]-feature_min[i]));
    }
    
    msg2.data = vector;
    featureVectors.push_back(vector);
    featureVector_pub.publish(msg2);
}
/*
void FeatureNormalizer::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    vector<float> vector = msg->data;
    featureVectors.push_back(vector);
    
    float current_min = *min_element(vector.begin(), vector.end());
    fv_min = current_min < fv_min ? current_min : fv_min;
    
    float current_max = *max_element(vector.begin(), vector.end());
    fv_max = current_max > fv_max ? current_max : fv_max;
}*/

void FeatureNormalizer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if(msg->status == "end") {
        if(output) {
            outputToFile(msg->name);
        }
        
        if(update) {
            storeParameters();
            update = false;
        }
        
        featureVectors.clear();
    }
}

/*
void FeatureNormalizer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if (msg->status == "end") {
        normalizeFeatureVectors();
        outputToFile(msg->name);
    }

    ROS_INFO("%s has %s", msg->name.c_str(), msg->status.c_str());
}
 */
 
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
    
    string path = music_dir + "/" + name + FV;
    file.open(path.c_str());
    
    transform(featureVectors.begin(), featureVectors.end(), featureVectors.begin(), output_m);
    
    file.close();
    
    //fv_min = 0.0;
    //fv_max = 0.0;
    
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
    return (fv_max-fv_min)==0 ? 0 : (val-fv_min)/(fv_max-fv_min);
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
