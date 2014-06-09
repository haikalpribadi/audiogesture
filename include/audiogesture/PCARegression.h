/* 
 * File:   PCARegression.h
 * Author: haikalpribadi
 *
 * Created on 25 May 2014, 16:07
 */

#ifndef PCAREGRESSION_H
#define	PCAREGRESSION_H

#include <algorithm>
#include <armadillo>
#include <ctime>
#include <dirent.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <mlpack/core.hpp>
#include <mlpack/methods/linear_regression/linear_regression.hpp>
#include <numeric>
#include <pca.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "audiogesture/ExtractorStatus.h"
#include "audiogesture/FeatureVector.h"
#include "audiogesture/GestureVector.h"
#include "audiogesture/OutputRecord.h"
#include "CompareNatural.h"

using namespace std;
using namespace mlpack::regression;

struct stat sb;

class PCARegression {
public:
    PCARegression();
    void setupNode();
    void loadDirectory();
    void process();

private:
    ros::NodeHandle node;
    ros::Subscriber featureVector_sub;
    ros::Subscriber extractorStatus_sub;
    ros::Subscriber recordStatus_sub;
    ros::Publisher outputVector_pub;
    ros::Publisher outputRecord_pub;
    
    stats::pca gesture_pca;
    stats::pca feature_pca;
    
    string pca_dir;
    string train_dir;
    string gesture_dir;
    string feature_dir;
    bool filter;
    bool record;
    int dimension;
    int gestureRows;
    int gestureCols;
    int featureSize;
    int gestureScale;
    double featureRate;
    double gestureRate;
    double gestureDelay;
    arma::mat correlationMatrix;
    arma::mat featureMagnitudeMatrix;
    arma::mat gestureMagnitudeMatrix;
    vector<LinearRegression> regressionModel;
    vector<string> featureFiles;
    vector<string> gestureFiles;
    vector<double> featureVector;
    vector<vector<double> > featureVectors;
    vector<vector<double> > featureMagnitudes;
    vector<vector<double> > gestureMagnitudes;
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
    void saveResults();
    void outputToFile(const vector<vector<double> >& data, const string& path);
    void featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg);
    void extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg);
    void recordCallback(const std_msgs::String::ConstPtr& msg);
    void mapFeatureToGesture();
    void solveScalarVectors();
    void solveCorrelationMatrix();
};

#endif	/* PCAREGRESSION_H */

