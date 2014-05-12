/* 
 * File:   GestureKinectInput.cpp
 * Author: haikalpribadi
 * 
 * Created on 07 May 2014, 15:56
 */


#include "GestureKinectInput.h"

GestureKinectInput::GestureKinectInput() {
    ROS_INFO("GestureKinectInput has started");
    if (node.getParam("filter_size_n", filterSizeN)) {
    } else {
        ;

    }
    gesture_pub = node.advertise<audiogesture::GestureVector>("gesture_kinect_input", 1000);
    depth_sub = node.subscribe("/camera/depth/points", 1000,
            &GestureKinectInput::depthCallback, this);
    calibrate_sub = node.subscribe("kinect_calibrate", 1000, 
            &GestureKinectInput::calibrateCallback, this);

    x_min = 0;
    x_max = 640;
    y_min = 0;
    y_max = 480;
    z_min = 0;
    z_max = 1000;
    node.param("kinect_x_min", x_min, x_min);
    node.param("kinect_x_max", x_max, x_max);
    node.param("kinect_y_min", y_min, y_max);
    node.param("kinect_y_max", y_max, y_max);
    node.param("kinect_z_min", z_min, z_min);
    node.param("kinect_z_max", z_max, z_max);
    
    calibrate = false;
    calibrated = false;
    calibrationSampleSize = 10;
    node.param("kinect_calibration_sample_size", calibrationSampleSize, calibrationSampleSize);
    
    smoothing = false;
    node.param("kinect_smoothing", smoothing, smoothing);
    
    sampleSize = 10;
    node.param("kinect_sample_size", sampleSize, sampleSize);
    
    filterSizeN = 5;
    node.param("kinect_filter_size_n", filterSizeN, filterSizeN);
    filterSize = 2 * filterSizeN + 1;
    sigma = (filterSizeN + 1) / 3.7;
    cout << "Using gaussian filter: ";
    for (int i = -1 * filterSizeN; i <= filterSizeN; i++) {
        float g = (1 / (sigma * sqrt(2 * M_PI))) * exp(-pow(i, 2) / (2 * pow(sigma, 2)));
        gaussianFilter.push_back(g);
        cout << g << ", ";
    }
    cout << endl;

    if (x_min < 0 || x_max > 640 || x_min >= x_max ||
            y_min < 0 || y_max > 480 || y_min >= y_max ||
            z_min < 0 || z_max > 1000 || z_min >= z_max) {
        ROS_ERROR("GestureKinectInput using INCORRECT margin values of: x_min: %d, x_max: %d, y_min: %d, y_max: %d, z_min: %d, z_max: %d",
                x_min, x_max, y_min, y_max, z_min, z_max);
    }

    ROS_INFO("GestureKinectInput using margin fiters of x_min: %d, x_max: %d, y_min: %d, y_max: %d, z_min: %d, z_max: %d",
            x_min, x_max, y_min, y_max, z_min, z_max);

}

void GestureKinectInput::calibrateCallback(const std_msgs::Empty::ConstPtr& msg) {
    ROS_INFO("GestureKinectInput will start calibration with current visible plane");
    
    calibrate = true;
    calibrated = false;
}

void GestureKinectInput::calibrateOffset() {
    offsetPoints.clear();
    
    for(int j=0; j<calibrationPoints[0].size(); j++) {
        float z = 0;
        for(int i=0; i<calibrationPoints.size(); i++) {
            z += calibrationPoints[i][j];
        }
        z = z / calibrationPoints.size();
        offsetPoints.push_back(z);
    }
    calibrationPoints.clear();
    
    calibrate = false;
    calibrated = true;
    ROS_INFO("GestureKinectInput has completed calibrating the current visible plane");
}

void GestureKinectInput::depthCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    counter = counter % 100;
    if (counter > 0) {
        counter++;
        return;
    }

    int height = min((int) (msg->height), y_max - y_min);
    int width = min((int) (msg->width), x_max - x_min);
    int size = height*width;

    int x, rows, columns;
    vector<float> step1;
    vector<float> step2;
    audiogesture::GestureVector gesture;
    sensor_msgs::Image image;
    pcl::PointCloud<pcl::PointXYZ> input;
    pcl::fromROSMsg(*msg, input);

    if (smoothing) {
        for (int i = y_min; i < y_max; i++) {
            for (int j = x_min; j < x_max; j++) {
                float s = 0;
                float coef = 0.0;
                for (int u = max(-1 * filterSizeN, y_min - i); u <= min(filterSizeN, y_max-1 - i); u++) {
                    x = (i + u) * input.width + j + x_min;
                    float z = isnan(input.points[x].z) ? -1000 : input.points[x].z * 100;
                    if(z<z_min || z>z_max)
                        z = -1000;
                    else {
                        s += z * gaussianFilter[u + filterSizeN];
                        coef += gaussianFilter[u + filterSizeN];
                    }
                }
                s = coef==0? -1000 : s/coef;
                step1.push_back(s);
            }
        }
        for (int i = 0; i < height; i++) {
            for (int j = 0; j < width; j++) {
                float s = 0;
                float coef = 0.0;
                for (int u = max(-1 * filterSizeN, 0 - i); u <= min(filterSizeN, width-1 - j); u++) {
                    float z = step1[i * width + j + u];
                    if(!(z<z_min || z>z_max)) {
                        s += z * gaussianFilter[u + filterSizeN];
                        coef += gaussianFilter[u + filterSizeN];
                    }
                }
                s = coef==0? -1000 : s/coef;
                step2.push_back(s);
            }
        }
        
        for (int i = sampleSize/2; i < height; i += sampleSize) {
            for (int j = sampleSize/2; j < width; j += sampleSize) {
                float z = step2[i * width + j];
                gesture.data.push_back(z);
            }
        }
    } else {
        for (int i = y_min+sampleSize/2; i < y_max; i += sampleSize) {
            for (int j = x_min+sampleSize/2; j < x_max; j += sampleSize) {
                x = i*input.width + j+x_min;
                float z = isnan(input.points[x].z) ? -1000 : input.points[x].z * 100;
                if(z<z_min || z>z_max)
                    z = -1000;
                gesture.data.push_back(z);
            }
        }
    }
    gesture.height = floor(((float)height / sampleSize) + 0.5);
    gesture.width = floor(((float)width / sampleSize) + 0.5);
    
    if(calibrated){
        for(int i=0; i<gesture.data.size(); i++) {
            float z = (gesture.data[i]-offsetPoints[i]) * -1;
            z = min(0.0, floor(z * 100 + 0.5) / 100);
            gesture.data[i] = z;
        }
    }
    
    gesture_pub.publish(gesture);
    
    if(calibrate) {
        calibrationPoints.push_back(gesture.data);
        if(calibrationPoints.size() == calibrationSampleSize) {
            calibrateOffset();
        }
    }
    
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "GestureKinectInput");

    GestureKinectInput input;

    ros::spin();

    return 0;
}
