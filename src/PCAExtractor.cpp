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
    
    vector<string> gestureFiles = readDirectory(gesture_dir);
    vector<string> featureFiles = readDirectory(feature_dir);
    
    cout << "====== Gesture Files ======" << endl;
    for(int i=0; i<gestureFiles.size(); i++) {
        cout << gestureFiles[i] << endl;
        gestures.push_back(filterPeaks(loadData(gesture_dir + "/" + gestureFiles[i])));
    }
    for(int i=0; i<gestures[2].size(); i++) {
        for(int j=0; j<gestures[2][0].size(); j++) {
            cout << gestures[2][i][j] << ", ";
        }
        cout << endl;
    }
    
    cout << endl;
    cout << "====== Feature Files ======" << endl;
    for(int i=0; i<featureFiles.size(); i++){
        cout << featureFiles[i] << endl;
        gestures.push_back(loadData(feature_dir + "/" + featureFiles[i]));
    }
    
}

vector<string> PCAExtractor::readDirectory(const string& path) {
    vector<string> result;
    dirent* de;
    DIR* dp;
    errno = 0;
    dp = opendir(path.empty() ? "." : path.c_str());
    if (dp) {
        while (true) {
            errno = 0;
            de = readdir(dp);
            if (de == NULL) break;
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

vector<vector<double > > PCAExtractor::filterPeaks(vector<vector<double> > data) {
    int max = 1440;
    for(int i=0; i<data.size(); i++) {
        for(int j=0; j<data[0].size(); j++) {
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

int main(int argc, char** argv) {
    ros::init(argc, argv, "PCAExtractor");

    PCAExtractor pca;

    ros::spin();
}