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
    ROS_INFO("PCARegression is set to use gesture of the size: %d by %d", gestureRows, gestureCols);
    
    featureSize = 62;
    node.getParam("feature_size", featureSize);
    ROS_INFO("PCARegression is set to use feature_size of size: %d", featureSize);
    
    featureRate = 86.1328;
    node.getParam("feature_sample_rate", featureRate);
    ROS_INFO("PCARegression is set to use feature_sample_rate of: %f", featureRate);
    
    gestureRate = 30.0;
    node.getParam("gesture_sample_rate", gestureRate);
    ROS_INFO("PCARegression is set to use gesture_sample_rate of: %f", gestureRate);
    
    gestureDelay = 0.0;
    node.getParam("gesture_delay", gestureDelay);
    ROS_INFO("PCARegression is set to use gesture_delay of: %f", gestureDelay);
    
    gestureScale = 1000;
    node.getParam("gesture_output_scale", gestureScale);
    ROS_INFO("PCARegression is set to use gesture_output_scale of: %f", gestureScale);
}

void PCARegression::setupNode() {
    outputVector_pub = node.advertise<audiogesture::GestureVector>("gesture_output", 1000);
    
    featureVector_sub = node.subscribe("feature_vector", 1000,
            &PCARegression::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &PCARegression::extractorStatusCallback, this);
}

void PCARegression::featureVectorCallback(const audiogesture::FeatureVector::ConstPtr& msg) {
    //vector<double> feature(msg->data.begin(), msg->data.end());
    //featureVectors.push_back(feature);
    featureVector = vector<double>(msg->data.begin(), msg->data.end());
}

void PCARegression::extractorStatusCallback(const audiogesture::ExtractorStatus::ConstPtr& msg) {
    if (msg->status == "end") {
        mapFeatureToGesture();
    }
}

void PCARegression::mapFeatureToGesture() {
    if(featureVector.size()==0)
        return;
    
    arma::mat inputScalar(1,dimension+1);
    inputScalar(0,0) = 0;
    
    for(int i=0; i<dimension; i++) {
        inputScalar(0, i+1) = inner_product(featureVector.begin(), featureVector.end(),
                                      feature_eigenvectors[i].begin(), 0.0);
    }
    
    arma::mat outputScalar = inputScalar * correlationMatrix;
    outputScalar = outputScalar * -1 * gestureScale;
    arma::vec outputVector(gestureRows*gestureCols);
    outputVector.fill(0);
    
    for(int i=0; i<dimension; i++) {
        arma::vec gestureEigenVector(gesture_eigenvectors[i]);
        outputVector = outputVector + outputScalar(i) * gestureEigenVector;
    }
    
    audiogesture::GestureVector msg;
    for(int i=0; i<outputVector.n_elem; i++) {
        msg.data.push_back(outputVector(i));
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
    
    ROS_INFO("================================================================================");
    ROS_INFO("Completed processing PCA extractions! Duration: %d seconds", (int)duration);
    ROS_INFO("================================================================================");
    
    solveScalarVectors();
    
    start = std::clock();
    solveCorrelationMatrix();
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    
    ROS_INFO("================================================================================");
    ROS_INFO("Completed regression to compute correlation Matrix! Duration: %d seconds", (int)duration);
    ROS_INFO("================================================================================");
    
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
    int gestureSize = gestures[0][0].size();
    gesture_pca = stats::pca(gestureSize);
    gesture_pca.set_do_bootstrap(true, 100);
    for(int i=0; i<gestures.size(); i++) {
        validFile = true;
        for(int j=0; j<gestures[i].size(); j++) {
            vector<double> gesture;
            for(int k=0; k<gestures[i][j].size(); k++) {
                gesture.push_back(gestures[i][j][k]);
            }
            
            if(gesture.size()==gestureSize) {
                gesture_pca.add_record(gesture);
            } else {
                validFile = false;
                ROS_ERROR("%s has invalid data size", gestureFiles[i].c_str());
            }
        }
    }
    
    ROS_INFO("LOADING PCA records for feature data ... (%d dimension)", featureSize);
    if(featureSize > features[0][0].size()){
        ROS_ERROR("Dimension of PCA records for feature data (%d) is less than intended feature_size (%d)", 
                features[0][0].size(), featureSize);
        ros::requestShutdown();
    }
    feature_pca = stats::pca(featureSize);
    feature_pca.set_do_bootstrap(true, 100);
    for(int i=0; i<features.size(); i++) {
        validFile = true;
        for(int j=0; validFile && j<features[i].size(); j++) {
            vector<double> feature;
            for(int k=0; k<min((int)features[i][j].size(), featureSize); k++) {
                feature.push_back(features[i][j][k]);
            }
            
            if(feature.size()==featureSize) {
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

void PCARegression::solveScalarVectors() {
    ROS_INFO("SOLVING scalar vectors ...");
    
    double fstep = featureRate / min(featureRate, gestureRate);
    double gstep = gestureRate / min(featureRate, gestureRate);
    double scalar;
    int findex, gindex, counter, total=0;
    int delay = (int)(gestureDelay * gestureRate);
    
    cout << "gesture delay (seconds and samples): " << gestureDelay << ", " << delay << endl;
    cout << "feature rate: " << featureRate << endl;
    cout << "feature step: " << fstep << endl;
    cout << "gesture rate: " << gestureRate << endl;
    cout << "gesture step: " << gstep << endl;
    
    string path = pca_dir + "/result/scalar";
    remove(path.c_str());
    ofstream file;
    file.open(path.c_str());
    
    for(int f=0; f<features.size(); f++) {
        for(int g=0; g<gestures.size(); g++) {
            counter = 0;
            file << "================================================================================================================" << endl;
            file << "Feature file " << f+1 << " correlating with Gesture file " << g+1 << endl;
            file << "================================================================================================================" << endl;
            for(int i=0; 
                    (i*fstep)<features[f].size() && 
                    (i*gstep + delay)<gestures[g].size(); 
                    i++) {
                file << "[";
                vector<double> fscalars;
                findex = (int)(i*fstep);
                for(int j=0; j<dimension; j++) {
                    scalar = inner_product(features[f][findex].begin(), features[f][findex].begin()+featureSize,
                                                  feature_eigenvectors[j].begin(), 0.0);
                    scalar = scalar / inner_product(feature_eigenvectors[j].begin(), feature_eigenvectors[j].end(),
                                                    feature_eigenvectors[j].begin(), 0.0);
                    //featureScals << scalar;
                    fscalars.push_back(scalar);
                    file << fixed << setprecision(5) << scalar << ", ";
                }
                //featureScals << arma::endr;
                featureScalars.push_back(fscalars);
                
                file << "] <----> [";
                
                vector<double> gscalars;
                gindex = (int)(i*gstep) + delay;
                for(int j=0; j<dimension; j++) {
                    scalar = inner_product(gestures[g][gindex].begin(), gestures[g][gindex].end(),
                                                  gesture_eigenvectors[j].begin(), 0.0);
                    scalar = scalar / inner_product(gesture_eigenvectors[j].begin(), gesture_eigenvectors[j].end(),
                                                    gesture_eigenvectors[j].begin(), 0.0);
                    //gestureScals << scalar;
                    gscalars.push_back(scalar);
                    file << fixed << setprecision(5) << scalar << ", ";
                }
                //gestureScals << arma::endr;
                gestureScalars.push_back(gscalars);
                
                file << "]" << endl;
                counter++;
            }
            file << "----------------------------------------------------------------------------------------------------------------" << endl;
            file << "Total number of correlation samples: " << counter << endl;
            file << endl << endl << endl;
            total += counter;
        }
    }
    featureScalarMatrix = arma::mat(total,dimension);
    gestureScalarMatrix = arma::mat(total,dimension);

    for(int i=0; i<total; i++) {
        for(int j=0; j<dimension; j++) {
            featureScalarMatrix(i,j) = featureScalars[i][j];
            gestureScalarMatrix(i,j) = gestureScalars[i][j];
        }
    }

    cout << "Feature Scalar Matrix: " << featureScalarMatrix.n_rows << " (rows) x " << featureScalarMatrix.n_cols << " (cols)" << endl;
    cout << "Gesture Scalar Matrix: " << gestureScalarMatrix.n_rows << " (rows) x " << gestureScalarMatrix.n_cols << " (cols)" << endl;
}

void PCARegression::solveCorrelationMatrix() {
    ROS_INFO("SOLVING correlation matrix ...");
    
    correlationMatrix = arma::mat(dimension+1, dimension);
    string path = pca_dir + "/result/correlation";
    remove(path.c_str());
    ofstream file;
    file.open(path.c_str());
    
    for(int i=0; i<dimension; i++) {
        arma::vec res = gestureScalarMatrix.col(i);
        LinearRegression regression(featureScalarMatrix.t(), res);
        regressionModel.push_back(regression);
        
        arma::vec parameters = regression.Parameters();
        for(int j=0; j<parameters.size(); j++) {
            if(j==0)
                correlationMatrix(j,i) = parameters(j);
            else
                correlationMatrix(j,i) = parameters(j);
            
            file << correlationMatrix(j,i) << ",";
        }
        file << endl;
    }
    file.close();
    
    cout << "Correlation Matrix: " << correlationMatrix.n_rows << " (rows) x " << correlationMatrix.n_cols << " (cols)" << endl;
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
