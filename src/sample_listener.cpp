/* 
 * File:   sample_listener.cpp
 * Author: haikalpribadi
 *
 * Created on 12 February 2014, 13:06
 */

#include <cstdlib>
#include <sys/inotify.h>
#include <ros/ros.h>


#define EVENT_SIZE      ( sizeof ( struct inotify_event ))
#define BUF_LEN         ( 1024 * ( EVENT_SIZE + NAME_MAX + 1 ))

using namespace std;

/*
 * 
 */
int main(int argc, char** argv) 
{
    ros::init(argc, argv, "MusicSampleListener");
    ros::NodeHandle node;
    
    int length, i = 0;
    int listener = inotify_init1(IN_NONBLOCK);
    int working_dir;
    char buffer[BUF_LEN];
    string dir;
    
    if(node.getParam("music_dir", dir) && listener >= 0 )
    {
        ROS_INFO("MusicSampleListener using music_dir: %s", dir.c_str());
        working_dir = inotify_add_watch(listener, dir.c_str(), IN_CREATE);
    }
    else if(listener < 0)
    {
        ROS_ERROR("Unable to listen to music directory");
        ros::requestShutdown();
    }
    else
    {
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }
    
    while(length = read(listener, buffer, BUF_LEN))
    {
        struct inotify_event *event = (struct inotify_event *) &buffer[i];
        if(event->len && !(event->mask & IN_ISDIR))
        {
            ROS_INFO("A music file was created %s", event->name);
        }
    }
    
    if(length<0)
    {
        ROS_ERROR("Error while reading file");
    }
    
    inotify_rm_watch(listener, working_dir);
    close(listener);
    
    return 0;
}

