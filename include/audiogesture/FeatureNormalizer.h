/* 
 * File:   FeatureNormalizer.h
 * Author: haikalpribadi
 *
 * Created on 04 March 2014, 17:57
 */

#ifndef FEATURENORMALIZER_H
#define	FEATURENORMALIZER_H

#include <algorithm>
#include <fstream>
#include <iostream>
#include <ros/ros.h>
#include <sstream>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/FeatureVector.h"

#define FV ".fv"

using namespace std;

class FeatureNormalizer {
public:
    FeatureNormalizer();
    
    static float fv_min;
    static float fv_max;
    static ofstream file;
    static string filename;
    
    static vector<float> normalize_m(vector<float> vector);
    static float normalize_v(float val);
    
    static vector<float> output_m(vector<float> vector);
    static float output_v(float val);
    
private:
    ros::NodeHandle node;
    ros::Subscriber featureVector_sub;
    ros::Subscriber extractorStatus_sub;
    
    
    string music_dir;
    
    vector<vector<float> > featureVectors;
    
    void featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg);
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    void normalizeFeatureVectors();
    void outputToFile(string name);
    
};



#endif	/* FEATURENORMALIZER_H */

