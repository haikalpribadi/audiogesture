/* 
 * File:   PCAExtractor.h
 * Author: haikalpribadi
 *
 * Created on 27 March 2014, 23:07
 */

#ifndef PCAEXTRACTOR_H
#define	PCAEXTRACTOR_H

#include <algorithm>
#include <dirent.h>
#include <pca.h>
#include <ros/ros.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <vector>

#include "CompareNatural.h"

using namespace std;

struct stat sb;

class PCAExtractor {
public:
    PCAExtractor();
    
private:
    ros::NodeHandle node;
    string pca_dir;
    string train_dir;
    string gesture_dir;
    string feature_dir;
    vector<vector<vector<double > > > gestures;
    vector<vector<vector<double > > > features;
    
    vector<string> readDirectory(const string& path);
    vector<vector<double> > loadData(const string& path);
};

#endif	/* PCAEXTRACTOR_H */

