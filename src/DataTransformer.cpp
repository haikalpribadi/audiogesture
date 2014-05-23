/* 
 * File:   DataTransformer.cpp
 * Author: haikalpribadi
 * 
 * Created on 23 May 2014, 19:36
 */

#include "DataTransformer.h"

DataTransformer::DataTransformer() : rate(30) {
    ROS_INFO("DataTransformer will listen to transform_file");
    
    data_dir = "";
    if(node.getParam("experiment_data_dir", data_dir)) {
        ROS_INFO("DataTransformer using experiment_data_dir: %s", data_dir.c_str());
    }
    scale = 1.4;
    if(node.getParam("transform_scale", scale)) {
        ROS_INFO("DataTransformer using scale: %f", scale);
    }
    /*
    if(node.getParam("visualize_file", filename)) {
        ROS_INFO("VisualizeData will visualize data from: %s", filename.c_str());
    } else {
        ROS_ERROR("Please set visualize_file parameter for VisualizeData");
        ros::requestShutdown();
    }
     */
    
    file_sub = node.subscribe("transform_file", 1000, &DataTransformer::fileCallback, this);
}


void DataTransformer::fileCallback(const std_msgs::String::ConstPtr& msg) {
    string filename = msg->data;
    
    if(data_dir != "") {
        filename = data_dir + "/" + filename;
    }
    
    transformFile(filename);
}

void DataTransformer::transformFile(string filename) {
    vector<vector<double> > data;
    string line;
    
    ifstream infile(filename.c_str());
    
    if(infile.is_open()) {
        while(getline(infile,line)) {
            vector<double> values;
            string val;
            istringstream stream(line);
            while(getline(stream, val, ',')) {
                float f = atof(val.c_str());
                f = floor(pow(f, scale) * 100 + 0.5) / 100;
                values.push_back(f);
            }
            data.push_back(values);
        }
        infile.close();
    } else {
        ROS_ERROR("DataTransformer unable to open file at: %s", filename.c_str());
        return;
    }
    
    ROS_INFO("DataTransformer start transforming: %s", filename.c_str());
    
    string name = filename.substr(0, filename.rfind("."));
    string path = name + "-2.gv";
    ofstream outfile;
    outfile.open(path.c_str());
    
    for(int i=0; i<data.size(); i++) {
        for(int j=0; j<data[i].size(); j++) {
            outfile << data[i][j] << ",";
        }
        outfile << endl;
    }
    
    outfile.close();
    ROS_INFO("DataTransformer finish transforming: %s", filename.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "VisualizeData");
    
    DataTransformer visdata;
    
    ros::spin();
    
    return 0;
}