/* 
 * File:   sample_listener.cpp
 * Author: haikalpribadi
 *
 * Created on 12 February 2014, 13:06
 */

#include <cstdlib>
#include <sys/inotify.h>
#include <signal.h>
#include <ros/ros.h>

#define EVENT_SIZE      ( sizeof ( struct inotify_event ))
#define BUF_LEN         ( 1024 * ( EVENT_SIZE + NAME_MAX + 1 ))
#define WATCH_FLAGS     ( IN_CREATE | IN_MOVED_TO )

using namespace std;

static bool run = true;

void signalCallback(int signal)
{
    run = false;
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "MusicSampleListener");
    ros::NodeHandle node;
    
    int i = 0;
    int descriptor = inotify_init();
    int watched_dir;
    char buffer[BUF_LEN];
    string dir;
    
    if(node.getParam("music_dir", dir) && descriptor >= 0 )
    {
        ROS_INFO("MusicSampleListener using music_dir: %s", dir.c_str());
        watched_dir = inotify_add_watch(descriptor, dir.c_str(), WATCH_FLAGS);
        
    }
    else if(descriptor < 0)
    {
        ROS_ERROR("Unable to listen to music directory");
        ros::requestShutdown();
    }
    else
    {
        ROS_ERROR("Please set the music_directory (file) parameter for extractor");
        ros::requestShutdown();
    }
    
    signal(SIGINT, signalCallback);
    
    while(run)
    {
        while(run && read(descriptor, buffer, BUF_LEN)>0)
        {
            struct inotify_event *event = (struct inotify_event *) &buffer[i];
            if(event->len && !(event->mask & IN_ISDIR))
            {
                ROS_INFO("A music file was entered: %s", event->name);
            }
        }
    }
    
    inotify_rm_watch(descriptor, watched_dir);
    close(descriptor);
    
    return 0;
}

