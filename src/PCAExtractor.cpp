/* 
 * File:   PCAExtractor.cpp
 * Author: haikalpribadi
 * 
 * Created on 27 March 2014, 23:07
 */

#include "PCAExtractor.h"

PCAExtractor::PCAExtractor() {
    if (node.getParam("pca_dir", pca_dir)) {
        chdir(pca_dir.c_str());
        if(stat("train", &sb) == 0 && S_ISDIR(sb.st_mode) &&
           stat("train/gesture", &sb) == 0 && S_ISDIR(sb.st_mode) &&
           stat("train/feature", &sb) == 0 && S_ISDIR(sb.st_mode)) {
            ROS_INFO("PCAExtractor is using pca_dir: %s", pca_dir.c_str());
            train_dir = pca_dir + "/train";
            gesture_dir = train_dir + "/gesture";
            feature_dir = train_dir + "/feature";
        } else {
            ROS_ERROR("'train/gesture and train/feature folders are not available in the given pca_dir: %s", pca_dir.c_str());
        }
    } else {
        ROS_ERROR("Please set the pca_dir parameter for PCAExtractor");
        ros::requestShutdown();
    }
    
    filter = false;
    if(node.getParam("filter_peaks", filter)) {
        if(filter) {
            ROS_INFO("PCAExtractor is set to filter peaks (errors) in data");
        }
    }
    
}

void PCAExtractor::process() {
    loadPCA();
    solvePCA();

    ROS_INFO("=====================================");
    ROS_INFO("Completed processing PCA extractions!");
    ROS_INFO("=====================================");
    
    savePCA();
}

void PCAExtractor::loadPCA() {
    ROS_INFO("LOADING PCA records for gesture data ... (%d x %d dimension)", gestures[0].size(), gestures[0][0].size());
    int dimension = gestures[0].size()*gestures[0][0].size();
    gesture_pca = stats::pca(dimension);
    
    for(int i=0; i<gestures.size(); i++) {
        vector<double> gesture;
        for(int j=0; j<gestures[i].size(); j++) {
            for(int k=0; k<gestures[i][j].size(); k++) {
                gesture.push_back(gestures[i][j][k]);
            }
        }
        if(gesture.size()==dimension) {
            gesture_pca.add_record(gesture);
        } else {
            ROS_ERROR("%s has invalid data size", gestureFiles[i].c_str());
        }
    }
    
    
    ROS_INFO("LOADING PCA records for feature data ... (%d dimension)", features[0][0].size());
    dimension = features[0][0].size();
    feature_pca = stats::pca(dimension);
    
    for(int i=0; i<features.size(); i++) {
        bool validFile = true;
        for(int j=0; validFile && j<features[i].size(); j++) {
            vector<double> feature;
            for(int k=0; k<features[i][j].size(); k++) {
                feature.push_back(features[i][j][k]);
            }
            if(feature.size()==dimension) {
                feature_pca.add_record(feature);
            } else {
                validFile = false;
                ROS_ERROR("%s has invalid data size", featureFiles[i].c_str());
            }
        }
    }
    
}

void PCAExtractor::solvePCA() {
    ROS_INFO("SOLVING PCA for gesture data ...");
    gesture_pca.solve();
    ROS_INFO("SOLVING PCA for feature data ...");
    feature_pca.solve();
}

void PCAExtractor::savePCA() {
    if(!(stat("result", &sb) == 0 && S_ISDIR(sb.st_mode))) {
        mkdir("result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    ROS_INFO("SAVING PCA results for gesture data ...");
    gesture_pca.save("result/gesture");
    ROS_INFO("SAVING PCA results for feature data ...");
    feature_pca.save("result/feature");
    ROS_INFO("=DONE=");
}

vector<string> PCAExtractor::readDirectory(const string& path) {
    vector<string> result;
    dirent* de;
    DIR* dp;
    errno = 0;
    dp = opendir(path.c_str());
    if (dp) {
        while (true) {
            errno = 0;
            de = readdir(dp);
            if (de == NULL) break;
            string name = path + "/" + string(de->d_name);
            if(stat(name.c_str(), &sb) == 0 && !S_ISDIR(sb.st_mode))
                result.push_back(string(de->d_name));
        }
        closedir(dp);
        sort(result.begin(), result.end(), comparenat);
    }
    return result;
}

vector<vector<double> > PCAExtractor::loadData(const string& path) {
    vector<vector<double> > data;
    ifstream file(path.c_str());
    string line;
    
    if(file.is_open()) {
        while(getline(file,line)) {
            vector<double> values;
            string val;
            istringstream stream(line);
            while(getline(stream, val, ',')) {
                values.push_back(atoi(val.c_str()));
            }
            data.push_back(values);
        }
        file.close();
    }
    
    return data;
}

void PCAExtractor::loadDirectory() {
    gestureFiles = readDirectory(gesture_dir);
    featureFiles = readDirectory(feature_dir);
    
    int count = 0;
    cout << "====== Gesture Files ======" << endl;
    for(int i=0; i<gestureFiles.size(); i++) {
        string path = gesture_dir + "/" + gestureFiles[i];
        cout << gestureFiles[i] << endl;
        vector<vector<double> > data;
        if(filter) {
            data = filterPeaks(loadData(gesture_dir + "/" + gestureFiles[i]));
            stringstream ss;
            ss << "gesture" << count << ".gv";
            gestureFiles[i] = ss.str();
            ss.str("");
            ss.clear();
            ss << gesture_dir << "/gesture" << count++ << ".gv";
            outputToFile(data, ss.str());
        } else {
            data = loadData(gesture_dir + "/" + gestureFiles[i]);
        }
        gestures.push_back(data);
    }
    cout << "Gesture sample size: " << gestures.size() << endl;
    cout << "===========================" << endl << endl;
    
    count = 0;
    cout << "====== Feature Files ======" << endl;
    for(int i=0; i<featureFiles.size(); i++){
        cout << featureFiles[i] << endl;
        features.push_back(loadData(feature_dir + "/" + featureFiles[i]));
    }
    cout << "Feature sample size: " << features.size() << endl;
    cout << "===========================" << endl << endl;
}

vector<vector<double > > PCAExtractor::filterPeaks(vector<vector<double> > data) {
    int max = 1440;
    for(int i=0; i<data.size(); i++) {
        for(int j=0; j<data[i].size(); j++) {
            if(data[i][j]==max) {
                vector<double> values;
                for(int a=1; a<=j; a++) {
                    if(data[i][j-a]!=max) {
                        values.push_back(data[i][j-a]);
                        break;
                    }
                }
                for(int a=1; a<data[i].size()-j; a++) {
                    if(data[i][j+a]!=max) {
                        values.push_back(data[i][j+a]);
                        break;
                    }
                }
                for(int a=1; a<=i; a++) {
                    if(data[i-a][j]!=max) {
                        values.push_back(data[i-a][j]);
                        break;
                    }
                }
                for(int a=1; a<data.size()-i; a++) {
                    if(data[i+a][j]!=max) {
                        values.push_back(data[i+a][j]);
                        break;
                    }
                }
                double average = 0;
                for(int a=0; a<values.size(); a++) {
                    average += values[a];
                }
                data[i][j] = values.size()>0 ? average/values.size() : average;
            }
        }
    }
    return data;
}

void PCAExtractor::outputToFile(const vector<vector<double> >& data, const string& path) {
    ofstream file;
    file.open(path.c_str());
    
    for(int i=0; i<data.size(); i++) {
        for(int j=0; j<data[i].size(); j++) {
            file << data[i][j];
            if(j<data[i].size()-1)
                file << ",";
            else
                file << endl;
        }
    }
    
    file.close();
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "PCAExtractor");

    PCAExtractor pca;
    pca.loadDirectory();
    pca.process();

    ros::spin();
}