/* 
 * File:   MusicCollectionGenerator.h
 * Author: haikalpribadi
 *
 * Created on 14 February 2014, 18:08
 */

#ifndef COLLECTIONGENERATOR_H
#define	COLLECTIONGENERATOR_H

#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>

using namespace std;

#define MF      ".mf"

class CollectionGenerator {
public:
    CollectionGenerator();
    
private:
    string music_dir;
    
    ros::NodeHandle node;
    ros::Subscriber collectionGenerator_sub;
    
    void collectionGeneratorCallback(const std_msgs::String::ConstPtr& msg);
};

#endif	/* COLLECTIONGENERATOR_H */

