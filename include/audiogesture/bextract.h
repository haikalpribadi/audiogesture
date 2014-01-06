/* 
 * File:   bextract.h
 * Author: haikalpribadi
 *
 * Created on 28 December 2013, 21:46
 */

#ifndef BEXTRACT_H
#define	BEXTRACT_H

#include <cstdio>
#include <cstdlib>
#include <marsyas/Collection.h>
#include <marsyas/MarSystemManager.h>
#include <marsyas/Accumulator.h>
#include <marsyas/Fanout.h>
#include <marsyas/CommandLineOptions.h>
#include <marsyas/TimeLine.h>
#include <marsyas/FileName.h>
//#include <marsyas/common.h>
#include <string>
#include <vector>

using namespace std;
using namespace Marsyas;


MarSystem* createExtractorFromFile();
MarSystem* createBeatHistogramFeatureNetwork();
MarSystem* createBEATextrator();
MarSystem* createSTFTextractor();
MarSystem* createMFCCextractor();
MarSystem* createSTFTMFCCextractor();
MarSystem* createSCFextractor();
MarSystem* createSFMextractor();
MarSystem* createSFMSCFextractor();
MarSystem* createLSPextractor();
MarSystem* createLPCCextractor();
int printUsage(string progName);
int printHelp(string progName);
void beatHistogramFeatures(MarSystem* beatTracker, string sfName, realvec& 
        beatfeatures);
void tempo_histoSumBands(MarSystem* total1, string sfName, realvec& 
        beatfeatures, realvec& iwin, realvec& estimate);
void bextract_trainStereoSPS(vector<Collection> cls, string classNames, 
        string wekafname, mrs_natural memSize);
void bextract_trainStereoSPSMFCC(vector<Collection> cls, string classNames,
	string wekafname, mrs_natural memSize);
void bextract_trainStereoMFCC(vector<Collection> cls, string classNames,
	string wekafname, mrs_natural memSize);
void bextract_trainADRessStereoSPSMFCC(vector<Collection> cls, string 
        classNames, string wekafname, mrs_natural memSize);
void bextract_trainAccumulator(vector<Collection> cls, mrs_natural label,
        string pluginName, string classNames, string wekafname, mrs_natural 
        memSize, string extractorStr, bool withBeatFeatures);
void bextract_train(vector<Collection> cls, Collection cl, mrs_natural label,
        string pluginName, string classNames,string wekafname,  mrs_natural 
        memSize, string extractorStr, string classifierName);
void selectClassifier(MarSystem *msys,string classifierName );
void selectFeatureSet(MarSystem *featExtractor);
void bextract_train_refactored(string pluginName,  string wekafname,
        mrs_natural memSize, string classifierName, mrs_bool single_vector);
void bextract_train_rmsilence(vector<Collection> cls, mrs_natural label,
        string pluginName, string classNames, string wekafname,  mrs_natural 
        memSize, string extractorStr, string classifierName);
int readCollection(Collection& l, string name);
void initOptions();
void loadOptions();
void bextract(vector<string> soundfiles, mrs_natural label, string pluginName,
        string classNames, string wekafname,  mrs_natural memSize, string 
        extractorStr, string classifierName);
void mirex_bextract();
void saivq_train_refactored(string pluginName, string wekafname, mrs_natural 
        memSize, string classifierName, mrs_bool single_vector);
int bextractor(vector<string> args);


#endif	/* BEXTRACT_H */

