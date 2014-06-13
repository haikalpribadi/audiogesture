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
    
    record = false;
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
    outputRecord_pub = node.advertise<audiogesture::OutputRecord>("output_record", 1000);
    magnitudeRecord_pub = node.advertise<audiogesture::MagnitudeRecord>("magnitude_record",1000);
    
    featureVector_sub = node.subscribe("feature_vector", 1000,
            &PCARegression::featureVectorCallback, this);
    extractorStatus_sub = node.subscribe("extractor_status", 1000,
            &PCARegression::extractorStatusCallback, this);
    recordStatus_sub = node.subscribe("record_status", 1000,
            &PCARegression::recordCallback, this);
}

void PCARegression::recordCallback(const std_msgs::String::ConstPtr& msg) {
    if(msg->data == "start") {
        record = true;
    } else if(msg->data == "stop") {
        record = false;
    }
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
    
    arma::mat inputScalar(1,dimension);
    
    for(int i=0; i<dimension; i++) {
        inputScalar(0, i) = inner_product(featureVector.begin(), featureVector.end(),
                                      feature_eigenvectors[i].begin(), 0.0);
    }
    
    arma::mat outputScalar = inputScalar * correlationMatrix;
    outputScalar = outputScalar * -1;
    arma::vec outputVector(gestureRows*gestureCols);
    outputVector.fill(0);
    
    for(int i=0; i<dimension; i++) {
        arma::vec gestureEigenVector(gesture_eigenvectors[i]);
        outputVector = outputVector + outputScalar(i) * gestureEigenVector;
    }
    
    audiogesture::GestureVector msg;
    for(int i=0; i<outputVector.n_elem; i++) {
        double dat = outputVector(i) > 0 ? outputVector(i) : -1*outputVector(i);
        dat = dat * gestureScale;
        dat = dat + 1;
        dat = log(dat)/log(1.4);
        dat = dat * gestureScale;
        //dat = pow(dat, 2)/4;
        msg.data.push_back(dat);
        //msg.data.push_back(outputVector(i));
    }
    
    msg.height = gestureRows;
    msg.width = gestureCols;
    outputVector_pub.publish(msg);
    
    if(record) {
        audiogesture::OutputRecord msg2;
        msg2.feature_data.insert(msg2.feature_data.begin(), featureVector.begin(), featureVector.end());
        msg2.gesture_data = msg.data;
        msg2.gesture_height = gestureRows;
        msg2.gesture_width = gestureCols;
        outputRecord_pub.publish(msg2);
        
        audiogesture::MagnitudeRecord msg3;
        for(int i=0; i<inputScalar.n_elem; i++)
            msg3.feature_data.push_back(inputScalar(i));
        for(int i=0; i<outputScalar.n_elem; i++)
            msg3.gesture_data.push_back(outputScalar(i));
        magnitudeRecord_pub.publish(msg3);
    }
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
    
    saveResults();
    
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
    
    vector<vector<double> > feature_out;
    path = pca_dir + "/result/featuremagnitude";
    remove(path.c_str());
    ofstream file2;
    file2.open(path.c_str());
    bool featuremagnitude_ext = false;
    
    vector<vector<double> > gesture_out;
    path = pca_dir + "/result/gesturemagnitude";
    remove(path.c_str());
    ofstream file3;
    file3.open(path.c_str());
    bool gesturemagnitude_ext = false;
    
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
                featureMagnitudes.push_back(fscalars);
                if(!featuremagnitude_ext)
                  feature_out.push_back(fscalars);
                
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
                gestureMagnitudes.push_back(gscalars);
                if(!gesturemagnitude_ext)
                  gesture_out.push_back(gscalars);
                
                file << "]" << endl;
                counter++;
            }
            featuremagnitude_ext = true;
            gesturemagnitude_ext = true;
            
            file << "----------------------------------------------------------------------------------------------------------------" << endl;
            file << "Total number of correlation samples: " << counter << endl;
            file << endl << endl << endl;
            total += counter;
        }
    }
    featureMagnitudeMatrix = arma::mat(total,dimension);
    gestureMagnitudeMatrix = arma::mat(total,dimension);

    for(int i=0; i<total; i++) {
        for(int j=0; j<dimension; j++) {
            featureMagnitudeMatrix(i,j) = featureMagnitudes[i][j];
            gestureMagnitudeMatrix(i,j) = gestureMagnitudes[i][j];
        }
    }
    
    for(int j=0; j<feature_out[0].size(); j++) {
      for(int i=0; i<feature_out.size(); i++) {
        file2 << fixed << setprecision(5) << feature_out[i][j] << ",";
      }
      file2 << endl;
    }
    file2.close();
    
    
    for(int j=0; j<gesture_out[0].size(); j++) {
      for(int i=0; i<gesture_out.size(); i++) {
        file3 << fixed << setprecision(5) << gesture_out[i][j] << ",";
      }
      file3 << endl;
    }
    file3.close();

    cout << "Feature Scalar Matrix: " << featureMagnitudeMatrix.n_rows << " (rows) x " << featureMagnitudeMatrix.n_cols << " (cols)" << endl;
    cout << "Gesture Scalar Matrix: " << gestureMagnitudeMatrix.n_rows << " (rows) x " << gestureMagnitudeMatrix.n_cols << " (cols)" << endl;
}

void PCARegression::solveCorrelationMatrix() {
    ROS_INFO("SOLVING correlation matrix ...");
    
    correlationMatrix = arma::mat(dimension, dimension);
    string path = pca_dir + "/result/correlation";
    remove(path.c_str());
    ofstream file;
    file.open(path.c_str());
    
    for(int i=0; i<dimension; i++) {
        arma::vec res = gestureMagnitudeMatrix.col(i);
        LinearRegression regression(featureMagnitudeMatrix.t(), res);
        regressionModel.push_back(regression);
        
        arma::vec parameters = regression.Parameters();
        for(int j=0; j<parameters.size()-1; j++) {
            correlationMatrix(j,i) = parameters(j+1);
            
            file << correlationMatrix(j,i) << ",";
        }
        file << endl;
    }
    file.close();
    
    cout << "Correlation Matrix: " << correlationMatrix.n_rows << " (rows) x " << correlationMatrix.n_cols << " (cols)" << endl;
}

void PCARegression::saveResults() {
    if(!(stat("result", &sb) == 0 && S_ISDIR(sb.st_mode))) {
        mkdir("result", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    }
    ROS_INFO("SAVING PCA results for gesture data ...");
    gesture_pca.save("result/gesture");
    ROS_INFO("SAVING PCA results for feature data ...");
    feature_pca.save("result/feature");
    ROS_INFO("=DONE=");
    
    string path = pca_dir + "/result/eigenfeature";
    remove(path.c_str());
    ofstream file;
    file.open(path.c_str());
    
    for(int i=0; i<feature_eigenvectors.size(); i++) {
        for(int j=0; j<feature_eigenvectors[i].size(); j++) {
            file << fixed << setprecision(5) << feature_eigenvectors[i][j] << ",";
        }
        file << endl;
    }
    file.close();
    
    path = pca_dir + "/result/eigengesture";
    remove(path.c_str());
    file.open(path.c_str());
    
    for(int i=0; i<gesture_eigenvectors.size(); i++) {
        for(int j=0; j<gesture_eigenvectors[i].size(); j++) {
            file << fixed << setprecision(5) << gesture_eigenvectors[i][j] << ",";
        }
        file << endl;
    }
    file.close();
    
    path = pca_dir + "/result/eigenvalue_feature";
    remove(path.c_str());
    file.open(path.c_str());
    for(int i=0; i<feature_pca.get_num_variables(); i++) {
      file << fixed << setprecision(5) << feature_pca.get_eigenvalue(i) << ",";
    }
    file.close();
    
    path = pca_dir + "/result/eigenvalue_gesture";
    remove(path.c_str());
    file.open(path.c_str());
    for(int i=0; i<gesture_pca.get_num_variables(); i++) {
      file << fixed << setprecision(5) << gesture_pca.get_eigenvalue(i) << ",";
    }
    file.close();
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
