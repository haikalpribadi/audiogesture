/* 
 * File:   PCAExtractor.h
 * Author: haikalpribadi
 *
 * Created on 27 March 2014, 23:07
 */

#ifndef PCAEXTRACTOR_H
#define	PCAEXTRACTOR_H

#include <algorithm>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <numeric>
#include <pca.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/FeatureVector.h"
#include "audiogesture/GestureVector.h"
#include "CompareNatural.h"

using namespace std;

struct stat sb;

class PCAExtractor {
public:
    PCAExtractor();
    void setupNode();
    void loadDirectory();
    void process();
    
private:
    ros::NodeHandle node;
    ros::Subscriber featureVector_sub;
    ros::Subscriber extractorStatus_sub;
    ros::Publisher outputVector_pub;
    
    stats::pca gesture_pca;
    stats::pca feature_pca;
    
    string pca_dir;
    string train_dir;
    string gesture_dir;
    string feature_dir;
    bool filter;
    int dimension;
    int gestureRows;
    int gestureCols;
    int featureDimension;
    vector<string> gestureFiles;
    vector<string> featureFiles;
    vector<vector<double> > featureVectors;
    vector<vector<double> > feature_eigenvectors;
    vector<vector<double> > gesture_eigenvectors;
    vector<vector<double> > correlation;
    vector<vector<vector<double> > > gestures;
    vector<vector<vector<double> > > features;
    
    vector<string> readDirectory(const string& path);
    vector<vector<double> > filterPeaks(vector<vector<double> > data);
    vector<vector<double> > loadData(const string& path);
    
    void loadPCA();
    void solvePCA();
    void savePCA();
    void outputToFile(const vector<vector<double> >& data, const string& path);
    void featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg);
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    void mapFeatureToGesture();
};

#endif	/* PCAEXTRACTOR_H */

