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
        ROS_INFO("DataTransformer using transform_scale: %f", scale);
    }
    reduction_x = 1;
    if(node.getParam("transform_reduction_x", reduction_x)) {
        ROS_INFO("DataTransformer using transform_reduction_x scale: %d", reduction_x);
    }
    reduction_y = 1;
    if(node.getParam("transform_reduction_y", reduction_y)) {
        ROS_INFO("DataTransformer using transform_reduction_y scale: %d", reduction_y);
    }
    cols = 48;
    if(node.getParam("transform_columns", cols)) {
        ROS_INFO("DataTransformer using transform_columns: %d", cols);
    } else {
        ROS_ERROR("DataTransofrmer requires transform_columns to be defined");
        ros::requestShutdown();
    }
    rows = 32;
    if(node.getParam("transform_rows", rows)) {
        ROS_INFO("DataTransformer using transform_rows: %d", rows);
    } else {
        ROS_ERROR("DataTransofrmer requires transform_rows to be defined");
        ros::requestShutdown();
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
    string path = msg->data;
    
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
                transformFile(path + "/" + string(de->d_name));
        }
        closedir(dp);
    } else {
        transformFile(path);
    }
}

void DataTransformer::transformFile(string filename) {
    vector<vector<double> > data;
    string line;
    float f;
    
    ifstream infile(filename.c_str());
    
    if(infile.is_open()) {
        while(getline(infile,line)) {
            vector<double> values;
            string val;
            istringstream stream(line);
            while(getline(stream, val, ',')) {
                f = atof(val.c_str());
                f = pow(f, scale);
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
    
    
    string name = filename.substr(filename.rfind("/"));
    string path = filename.substr(0, filename.rfind("/"));
    stringstream folder;
    folder << path << "/transformed_" << (int)(rows/reduction_y) << "x" << (int)(cols/reduction_x);
    remove(folder.str().c_str());
    mkdir(folder.str().c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    
    path = folder.str() + name;
    ofstream outfile;
    outfile.open(path.c_str());
    
    int j;
    float val;
    for(int i=0; i<data.size(); i++) {
        for(int y=0; y<rows; y+=reduction_y) {
            for(int x=0; x<cols; x+=reduction_x) {
                val = 0;
                for(int a=0; a<reduction_y; a++) {
                    for(int b=0; b<reduction_x; b++) {
                        j = (y+a)*cols + (x+b);
                        val += data[i][j];
                    }
                }
                val = val / (reduction_x*reduction_y);
                val = floor(val * 100 + 0.5) / 100;
                outfile << val << ",";
            }
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