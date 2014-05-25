/* 
 * File:   PCARegression.cpp
 * Author: haikalpribadi
 * 
 * Created on 25 May 2014, 16:07
 */

#include "PCARegression.h"

PCARegression::PCARegression() {
    if (node.getParam("pca_dir", pca_dir)) {
        chdir(pca_dir.c_str());
        if(stat("train", &sb) == 0 && S_ISDIR(sb.st_mode) &&
           stat("train/gesture", &sb) == 0 && S_ISDIR(sb.st_mode) &&
           stat("train/feature", &sb) == 0 && S_ISDIR(sb.st_mode)) {
            ROS_INFO("PCARegression is using pca_dir: %s", pca_dir.c_str());
            train_dir = pca_dir + "/train";
            gesture_dir = train_dir + "/gesture";
            feature_dir = train_dir + "/feature";
        } else {
            ROS_ERROR("'train/gesture and train/feature folders are not available in the given pca_dir: %s", pca_dir.c_str());
        }
    } else {
        ROS_ERROR("Please set the pca_dir parameter for PCARegression");
        ros::requestShutdown();
    }
    
    filter = false;
    if(node.getParam("filter_peaks", filter)) {
        if(filter) {
            ROS_INFO("PCARegression is set to filter peaks (errors) in data");
        }
    }
    dimension = 3;
    node.getParam("pca_dimension", dimension);
    ROS_INFO("PCARegression is set to use %d highest dimension", dimension);
    
    gestureRows = 4;
    gestureCols = 8;
    node.getParam("gesture_rows", gestureRows);
    node.getParam("gesture_cols", gestureCols);
    ROS_INFO("PCAExtractor is set to use gesture of the size: %d by %d", gestureRows, gestureCols);
    
    featureDimension = 62;
    node.getParam("feature_set", featureDimension);
    ROS_INFO("PCAExtractor is set to use feature_set of size: %d", featureDimension);
}

void PCARegression::setupNode() {
    outputVector_pub = node.advertise<audiogesture::GestureVector>("gesture_output", 1000);
    
    featureVector_sub = node.subscribe("feature_vector", 1000,
            &PCARegression::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &PCARegression::extractorStatusCallback, this);
}

void PCARegression::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    vector<double> feature(msg->data.begin(), msg->data.end());
    featureVectors.push_back(feature);
}

void PCARegression::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if (msg->status == "end") {
        mapFeatureToGesture();
    }
}

void PCARegression::mapFeatureToGesture() {
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
    
    vector<vector<double> > gesture_output;
    for(int i=0; i<gestureRows; i++) {
        vector<double> v;
        for(int j=0; j<gestureCols; j++) {
            v.push_back(0.0);
        }
        gesture_output.push_back(v);
    }
    int size = gesture_eigenvectors[0].size();
    for(int i=0; i<dimension; i++) {
        for(int j=0; j<size; j++) {
            int x = j/gestureCols;
            int y = j - (x*gestureCols);
            gesture_output[x][y] += (gesture_eigenvectors[i][j] * scalars[i]);
        }
    }
    vector<vector<double> > output;
    for(int i=0; i<gestureRows; i++) {
        vector<double> v;
        for(int j=0; j<gestureCols; j++) {
            double sum = 0.0;
            for(int x=max(0,i-1); x<=min(3,i+1); x++) {
                for(int y=max(0,j-1); y<=min(7,j+1); y++) {
                    if(x!=i || y!=j) {
                        sum += gesture_output[x][y];
                    }
                }
            }
            double average = sum / gestureCols;
            v.push_back(gesture_output[i][j] + average);
        }
        output.push_back(v);
    }
    
    audiogesture::GestureVector msg;
    for(int i=0; i<gestureRows; i++) {
        for(int j=0; j<gestureCols; j++) {
            msg.data.push_back(output[i][j]);
        }
    }
    msg.height = gestureRows;
    msg.width = gestureCols;
    outputVector_pub.publish(msg);
}

void PCARegression::process() {
    std::clock_t start;
    double duration;
    start = std::clock();
    
    node.setParam("pca_complete", false);
    loadPCA();
    solvePCA();
    node.setParam("pca_complete", true);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    
    ROS_INFO("=====================================");
    ROS_INFO("Completed processing PCA extractions! Duration: %f", duration);
    ROS_INFO("=====================================");
    
    savePCA();
    
}

void PCARegression::loadPCA() {
    ROS_INFO("LOADING PCA records for gesture data ... (%d dimension)", gestures[0][0].size());
    if(gestureRows*gestureCols != gestures[0][0].size()){
        ROS_ERROR("Dimension of PCA records for gesture data (%d) does not match intended gesture_rows (%d) x gesture_cols (%d)", 
                gestures[0][0].size(), gestureRows, gestureCols);
        ros::requestShutdown();
    }
    bool validFile = true;
    int dimension = gestures[0][0].size();
    gesture_pca = stats::pca(dimension);
    gesture_pca.set_do_bootstrap(true, 100);
    for(int i=0; i<gestures.size(); i++) {
        validFile = true;
        for(int j=0; j<gestures[i].size(); j++) {
            vector<double> gesture;
            for(int k=0; k<gestures[i][j].size(); k++) {
                gesture.push_back(gestures[i][j][k]);
            }
            
            if(gesture.size()==dimension) {
                gesture_pca.add_record(gesture);
            } else {
                validFile = false;
                ROS_ERROR("%s has invalid data size", gestureFiles[i].c_str());
            }
        }
    }
    
    ROS_INFO("LOADING PCA records for feature data ... (%d dimension)", featureDimension);
    if(featureDimension > features[0][0].size()){
        ROS_ERROR("Dimension of PCA records for feature data (%d) is less than intended feature_set (%d)", 
                features[0][0].size(), featureDimension);
        ros::requestShutdown();
    }
    dimension = featureDimension;
    feature_pca = stats::pca(dimension);
    feature_pca.set_do_bootstrap(true, 100);
    for(int i=0; i<features.size(); i++) {
        validFile = true;
        for(int j=0; validFile && j<features[i].size(); j++) {
            vector<double> feature;
            for(int k=0; k<min((int)features[i][j].size(), dimension); k++) {
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

void PCARegression::solvePCA() {
    ROS_INFO("SOLVING PCA for gesture data ...");
    gesture_pca.solve();
    for(int i=0; i<dimension; i++)
        gesture_eigenvectors.push_back(gesture_pca.get_eigenvector(i));
    
    ROS_INFO("SOLVING PCA for feature data ...");
    feature_pca.solve();
    for(int i=0; i<dimension; i++)
        feature_eigenvectors.push_back(feature_pca.get_eigenvector(i));
    
}

void PCARegression::savePCA() {
    if(!(stat("result", &sb) == 0 && S_ISDIR(sb.st_mode))) {
        mkdir("result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    ROS_INFO("SAVING PCA results for gesture data ...");
    gesture_pca.save("result/gesture");
    ROS_INFO("SAVING PCA results for feature data ...");
    feature_pca.save("result/feature");
    ROS_INFO("=DONE=");
}

vector<string> PCARegression::readDirectory(const string& path) {
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

vector<vector<double> > PCARegression::loadData(const string& path) {
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

void PCARegression::loadDirectory() {
    gestureFiles = readDirectory(gesture_dir);
    featureFiles = readDirectory(feature_dir);
    
    for(int i=0; i<gestureFiles.size(); i++) {
        string path = gesture_dir + "/" + gestureFiles[i];
        gestures.push_back(loadData(gesture_dir + "/" + gestureFiles[i]));
    }
    ROS_INFO("Gesture sample size: %d", gestures.size());
    
    for(int i=0; i<featureFiles.size(); i++){
        features.push_back(loadData(feature_dir + "/" + featureFiles[i]));
    }
    ROS_INFO("Feature sample size: %d", features.size());
}

vector<vector<double > > PCARegression::filterPeaks(vector<vector<double> > data) {
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

void PCARegression::outputToFile(const vector<vector<double> >& data, const string& path) {
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
    ros::init(argc, argv, "PCARegression");

    PCARegression pca;
    pca.loadDirectory();
    pca.process();
    pca.setupNode();

    ros::spin();
    
    return 0;
}
