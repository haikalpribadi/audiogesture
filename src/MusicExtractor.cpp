/* 
 * File:   extractor.cpp
 * Author: haikalpribadi
 *
 * Created on 29 December 2013, 16:08
 */

#include "MusicExtractor.h"

MusicExtractor::MusicExtractor() {
    if (node.getParam("music_dir", music_dir)) {
        ROS_INFO("MusicFeatureExtractor using music_dir: %s", music_dir.c_str());
        chdir(music_dir.c_str());
    } else {
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }
    extractor_sub = node.subscribe("music_extractor", 1000,
            &MusicExtractor::extractorCallback, this);
    extractorStatus_pub = node.advertise<audiogesture::ExtractorStatus>("extractor_status", 1000);
}

void MusicExtractor::extractorCallback(const std_msgs::String::ConstPtr& msg) {
    istringstream iss(msg->data.c_str());
    vector<string> tokens;
    copy(istream_iterator<string>(iss),
            istream_iterator<string>(),
            back_inserter<vector<string> >(tokens));

    audiogesture::ExtractorStatus status;
    status.name = tokens[0].substr(0, tokens[0].find("."));
    status.status = "start";
    extractorStatus_pub.publish(status);

    bextractor(tokens);

    status.status = "end";
    extractorStatus_pub.publish(status);

    ROS_INFO("BEXTRACTOR CALLED");
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "MusicFeatureExtractor");

    MusicExtractor musicExtractor;

    ros::spin();
    return 0;
}
