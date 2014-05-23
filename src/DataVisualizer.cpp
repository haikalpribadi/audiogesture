/* 
 * File:   DataVisualizer.cpp
 * Author: haikalpribadi
 * 
 * Created on 23 May 2014, 18:10
 */

#include "DataVisualizer.h"

DataVisualizer::DataVisualizer() {
    ROS_INFO("DataVisualizer will listen to visualize_file");
    
    data_dir = "";
    if(node.getParam("experiment_data_dir", data_dir)) {
        ROS_INFO("VisualizerData using experiment_data_dir: %s", data_dir.c_str());
    }
    /*
    if(node.getParam("visualize_file", filename)) {
        ROS_INFO("VisualizeData will visualize data from: %s", filename.c_str());
    } else {
        ROS_ERROR("Please set visualize_file parameter for VisualizeData");
        ros::requestShutdown();
    }
     */
    
    file_sub = node.subscribe("visualize_file", 1000, &DataVisualizer::fileCallback, this);
    data_pub = node.advertise<audiogesture::GestureVector>("visualize_data", 1000);
}


void DataVisualizer::fileCallback(const audiogesture::DataFile::ConstPtr& msg) {
    ros::Rate rate(30);
    vector<vector<double> > data;
    string filename = msg->name;
    string line;
    
    if(data_dir != "") {
        filename = data_dir + "/" + filename;
    }
    
    ifstream file(filename.c_str());
    
    if(file.is_open()) {
        while(getline(file,line)) {
            vector<double> values;
            string val;
            istringstream stream(line);
            while(getline(stream, val, ',')) {
                values.push_back(atof(val.c_str()));
            }
            data.push_back(values);
        }
        file.close();
    } else {
        ROS_ERROR("VisualizeData unable to open file at: %s", filename.c_str());
        return;
    }
    
    ROS_INFO("DataVisualizer start visualizing: %s", filename.c_str());
    
    for(int i=0; i<data.size() && ros::ok(); i++) {
        audiogesture::GestureVector msg2;
        msg2.data.insert(msg2.data.begin(), data[i].begin(), data[i].end());
        msg2.height = msg->height;
        msg2.width = msg->width;
        data_pub.publish(msg2);
        
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "VisualizeData");
    
    DataVisualizer visdata;
    
    ros::spin();
    
    return 0;
}