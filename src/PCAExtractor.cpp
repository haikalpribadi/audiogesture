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
    dimension = 3;
    node.getParam("pca_dimension", dimension);
    ROS_INFO("PCAExtractor is set to use %d highest dimension", dimension);
}

void PCAExtractor::setupNode() {
    outputVector_pub = node.advertise<std_msgs::Float64MultiArray>("gesture_output", 1000);
    
    featureVector_sub = node.subscribe("feature_vector", 1000,
            &PCAExtractor::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &PCAExtractor::extractorStatusCallback, this);
}

void PCAExtractor::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    vector<double> feature(msg->data.begin(), msg->data.end());
    featureVectors.push_back(feature);
}

void PCAExtractor::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if (msg->status == "end") {
        mapFeatureToGesture();
    }
}

void PCAExtractor::mapFeatureToGesture() {
    if(featureVectors.size()<1)
        return;
    
    stats::pca input_pca(featureVectors[0].size());
    input_pca.set_do_bootstrap(true, 100);
    
    for(int i=0; i<featureVectors.size(); i++) {
        input_pca.add_record(featureVectors[i]);
    }
    featureVectors.clear();
    
    input_pca.solve();
    
    vector<double> input_ev = input_pca.get_eigenvector(0);
    vector<double> scalars;
    
    for(int i=0; i<dimension; i++) {
        double scalar = inner_product(input_ev.begin(), input_ev.end(), 
                                      feature_eigenvectors[i].begin(), 0.0) * 5;
        scalars.push_back(scalar);
    }
    cout << endl;
    
    /*
    int size = gesture_eigenvectors[0].size();
    vector<double> gesture_output(size);
    for(int i=0; i<size; i++)
        gesture_output[i] = 0.0;
    
    for(int i=0; i<dimension; i++) {
        for(int j=0; j<size; j++) {
            gesture_output[j] += (gesture_eigenvectors[i][j] * scalars[i]);
        }
    }
    
    std_msgs::Float64MultiArray msg;
    for(int i=0; i<gesture_output.size(); i++) {
        msg.data.push_back(gesture_output[i]);
    }
    outputVector_pub.publish(msg);
    */
    
    vector<vector<double> > gesture_output;
    for(int i=0; i<4; i++) {
        vector<double> v;
        for(int j=0; j<8; j++) {
            v.push_back(0.0);
        }
        gesture_output.push_back(v);
    }
    int size = gesture_eigenvectors[0].size();
    for(int i=0; i<dimension; i++) {
        for(int j=0; j<size; j++) {
            int x = j/8;
            int y = j - (x*8);
            gesture_output[x][y] += (gesture_eigenvectors[i][j] * scalars[i]);
        }
    }
    vector<vector<double> > output;
    for(int i=0; i<4; i++) {
        vector<double> v;
        for(int j=0; j<8; j++) {
            double sum = 0.0;
            for(int x=max(0,i-1); x<=min(3,i+1); x++) {
                for(int y=max(0,j-1); y<=min(7,j+1); y++) {
                    if(x!=i || y!=j) {
                        sum += gesture_output[x][y];
                    }
                }
            }
            double average = sum / 8;
            v.push_back(gesture_output[i][j] + average);
        }
        output.push_back(v);
    }
    
    std_msgs::Float64MultiArray msg;
    for(int i=0; i<4; i++) {
        for(int j=0; j<8; j++) {
            msg.data.push_back(output[i][j]);
        }
    }
    outputVector_pub.publish(msg);
}

void PCAExtractor::process() {
    node.setParam("pca_complete", false);
    loadPCA();
    solvePCA();
    node.setParam("pca_complete", true);

    ROS_INFO("=====================================");
    ROS_INFO("Completed processing PCA extractions!");
    ROS_INFO("=====================================");
    
    savePCA();
    
}

void PCAExtractor::loadPCA() {
    ROS_INFO("LOADING PCA records for gesture data ... (%d x %d dimension)", gestures[0].size(), gestures[0][0].size());
    int dimension = gestures[0].size()*gestures[0][0].size();
    gesture_pca = stats::pca(dimension);
    gesture_pca.set_do_bootstrap(true, 100);
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
    feature_pca.set_do_bootstrap(true, 100);
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
    for(int i=0; i<dimension; i++)
        gesture_eigenvectors.push_back(gesture_pca.get_eigenvector(i));
    
    ROS_INFO("SOLVING PCA for feature data ...");
    feature_pca.solve();
    for(int i=0; i<dimension; i++)
        feature_eigenvectors.push_back(feature_pca.get_eigenvector(i));
    
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
    //ROS_INFO("====== Gesture Files ======");
    for(int i=0; i<gestureFiles.size(); i++) {
        string path = gesture_dir + "/" + gestureFiles[i];
        //ROS_INFO(gestureFiles[i].c_str());
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
    ROS_INFO("Gesture sample size: %d", gestures.size());
    
    count = 0;
    //ROS_INFO("====== Feature Files ======");
    for(int i=0; i<featureFiles.size(); i++){
        //ROS_INFO(featureFiles[i].c_str());
        features.push_back(loadData(feature_dir + "/" + featureFiles[i]));
    }
    ROS_INFO("Feature sample size: %d", features.size());
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
    pca.setupNode();

    ros::spin();
    
    return 0;
}