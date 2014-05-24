/* 
 * File:   FeatureNormalizer.h
 * Author: haikalpribadi
 *
 * Created on 04 March 2014, 17:57
 */

#ifndef FEATURENORMALIZER_H
#define	FEATURENORMALIZER_H

#include <algorithm>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/FeatureVector.h"
#include "CompareNatural.h"

#define FV ".fv"

struct stat sb;
using namespace std;

class FeatureNormalizer {
public:
    FeatureNormalizer();
    
    static float fv_min;
    static float fv_max;
    static ofstream file;
    
    static vector<float> normalize_m(vector<float> vector);
    static float normalize_v(float val);
    
    static vector<float> output_m(vector<float> vector);
    static float output_v(float val);
    
private:
    ros::NodeHandle node;
    ros::Publisher featureVector_pub;
    ros::Subscriber featureVector_sub;
    ros::Subscriber extractorStatus_sub;
    
    vector<float> feature_min;
    vector<float> feature_max;
    
    bool initialize;
    bool output;
    bool update;
    bool updateRange;
    string music_dir;
    string parameter_dir;
    string parameter_file;
    string args;
    
    vector<vector<float> > featureVectors;
    
    void featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg);
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    void normalizeFeatureVectors();
    void outputToFile(string name);
    
    void loadParameters();
    void storeParameters();
};



#endif	/* FEATURENORMALIZER_H */

