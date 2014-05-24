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
    scale = 1.0;
    if(node.getParam("transform_scale", scale)) {
        ROS_INFO("DataVisualizer using scale: %f", scale);
    }
    amp = 1.0;
    if(node.getParam("transform_mult", amp)) {
        ROS_INFO("DataVisualizer using amp: %f", amp);
    }
    rate = 30;
    if(node.getParam("visualizer_data_rate", rate)) {
        ROS_INFO("DataVisualizer using rate: %d", rate);
    }
    normalize = false;
    node.getParam("visualizer_normalize", normalize);
    
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
    string path = msg->name;
    
    if(data_dir != "") {
        path = data_dir + "/" + path;
    }
    
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
                visualizeFile(path + "/" + string(de->d_name), msg->height, msg->width);
        }
        closedir(dp);
    } else {
        visualizeFile(path, msg->height, msg->width);
    }
}

void DataVisualizer::visualizeFile(string filename, int height, int width) {
    ros::Rate ros_rate(rate);
    vector<vector<double> > data;
    string line;
    float f;
    
    ifstream file(filename.c_str());
    float min=0, max=0;
    if(file.is_open()) {
        while(getline(file,line)) {
            vector<double> values;
            string val;
            istringstream stream(line);
            while(getline(stream, val, ',')) {
                f = atof(val.c_str());
                f = floor(pow(f, scale) * 100 + 0.5) / 100;
                f = f*amp;
                values.push_back(f);
                if(normalize) {
                    min = f<min ? f : min;
                    max = f>max ? f : max;
                }
            }
            data.push_back(values);
        }
        file.close();
    } else {
        ROS_ERROR("VisualizeData unable to open file at: %s", filename.c_str());
        return;
    }
    
    if(normalize) {
        for(int i=0; i<data.size(); i++) {
            for(int j=0; j<data.size(); j++) {
                if(max-min==0){
                    data[i][j] = 0;
                } else {
                    data[i][j] = (data[i][j]-min)/(max-min)*10;
                }
            }
        }
    }
    ROS_INFO("DataVisualizer start visualizing: %s", filename.c_str());
    
    for(int i=0; i<data.size() && ros::ok(); i++) {
        audiogesture::GestureVector msg;
        msg.data.insert(msg.data.begin(), data[i].begin(), data[i].end());
        msg.height = height;
        msg.width = width;
        data_pub.publish(msg);
        
        ros_rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "VisualizeData");
    
    DataVisualizer visdata;
    
    ros::spin();
    
    return 0;
}