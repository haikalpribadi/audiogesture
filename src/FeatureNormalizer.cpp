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

    features = 1000;
    if (node.getParam("feature_size", features)) {
        ROS_INFO("FeatureNormalizer will process and produce %d number of features", features);
    }
    
    if (!node.getParam("bextract_args", args)) {
        args = "";
    }
    parameter_file = "ranges" + args + ".txt";
    
    output = true;
    node.getParam("output_to_file", output);
    
    updateRange = true;
    node.getParam("normalizer_update_range", updateRange);
    
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
    
    int i;
    string path = parameter_dir + "/" + parameter_file;
    string line, val;
    ifstream file(path.c_str());
    if(file.good() && file.is_open()) {
        while(getline(file,line)) {
            istringstream stream(line);
            i = 1;
            while(getline(stream, val, ',')) {
                if(i==1)
                    feature_min.push_back(atof(val.c_str()));
                else if(i==2)
                    feature_max.push_back(atof(val.c_str()));
                i++;
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
    
    for(int i=0; i<feature_min.size() && i<feature_max.size(); i++) {
        file << (floor(feature_min[i] * 100 + 0.5) / 100) << ",";
        file << (floor(feature_max[i] * 100 + 0.5) / 100) << ",";
        file << endl;
    }
    
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
    
    for(int i=0; i<msg->data.size() && i<features; i++) {
        if(updateRange) {
            if(msg->data[i] < feature_min[i]) {
                feature_min[i] = msg->data[i];
                update = true;
            }
            if(msg->data[i] > feature_max[i]) {
                feature_max[i] = msg->data[i];
                update = true;
            }
        }
        
        if(feature_max[i]-feature_min[i] == 0) {
            vector.push_back(0);
        } else if (!updateRange) {
            vector.push_back(min(1.0f,max(0.0f,(msg->data[i]-feature_min[i])/(feature_max[i]-feature_min[i]))));
        } else {
            vector.push_back((msg->data[i]-feature_min[i])/(feature_max[i]-feature_min[i]));
        }
    }
    
    msg2.data = vector;
    featureVectors.push_back(vector);
    featureVector_pub.publish(msg2);
}

void FeatureNormalizer::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if(msg->status == "end") {
        if(output) {
            outputToFile(msg->name);
        }
        
        if(update && updateRange) {
            storeParameters();
            update = false;
        } else if(update) {
            ROS_WARN("Feature Normalization range updated!!");
        }
        
        featureVectors.clear();
    }
}
 
void FeatureNormalizer::normalizeFeatureVectors() {
    if(featureVectors.size()<1)
        return;
    
    transform(featureVectors.begin(), featureVectors.end(), featureVectors.begin(), normalize_m);
}

void FeatureNormalizer::outputToFile(string name) {
    if(featureVectors.size()<1)
        return;
    
    string path = music_dir + "/" + name + FV;
    file.open(path.c_str());
    
    transform(featureVectors.begin(), featureVectors.end(), featureVectors.begin(), output_m);
    
    file.close();
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
