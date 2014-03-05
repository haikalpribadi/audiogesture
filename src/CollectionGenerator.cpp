/* 
 * File:   MusicCollectionGenerator.cpp
 * Author: haikalpribadi
 * 
 * Created on 14 February 2014, 18:08
 */

#include "CollectionGenerator.h"

CollectionGenerator::CollectionGenerator() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("MusicCollectionGenerator using music_dir: %s", music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for MusicCollectionGenerator");
        ros::requestShutdown();
    }

    collectionGenerator_sub = node.subscribe("collection_generator", 1000,
            &CollectionGenerator::collectionGeneratorCallback, this);
}

void CollectionGenerator::collectionGeneratorCallback(const std_msgs::String::ConstPtr& msg) {
    string sample = msg->data;
    string name = sample.substr(0, sample.find(".")) + MF;

    replace(name.begin(), name.end(), ' ', '_');
    name = music_dir + "/" + name;

    FILE * file;
    file = fopen(name.c_str(), "w");
    if (file != NULL) {
        fputs(sample.c_str(), file);
        fclose(file);
    }

    return;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "MusicCollectionGenerator");

    CollectionGenerator collectionGenerator;

    ros::spin();
    return 0;
}