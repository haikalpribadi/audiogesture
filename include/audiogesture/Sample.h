/* 
 * File:   Sample.h
 * Author: haikalpribadi
 *
 * Created on 09 March 2014, 15:00
 */

#ifndef SAMPLE_H
#define	SAMPLE_H

#include <cstdlib>
#include <string>
#include <vector>

using namespace std;

class Sample {
public:
    Sample();
    Sample(string sname);
    
    string name;
    
    string sampleFile;
    string collectionFile;
    string featureFile;
    string featureNormFile;
    
    vector<string> gestureFiles;
    
    void setSample(string filename);
    void setCollection(string filename);
    void setFeature(string filename);
    void setFeatureNorm(string filename);
    void addGestureFile(string filename);
    
private:

};

#endif	/* SAMPLE_H */

