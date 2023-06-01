// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

#include "lidar.h"


lidar::Lidar::Lidar(){
    // initialize with default values
    max_distance = 1.;
    min_distance = 30.;

    scan_period = 0.1;

    horizontal_angle = 120.;
    horizontal_angle_resolution = 0.1;
    min_horizontal_angle = -60.;
    max_horizontal_angle = +60.;

    vertical_angle = 32.;
    vertical_angle_resolution = 0.1;
    min_vertical_angle = -16.;
    max_vertical_angle = +16.;

    num_lines = 16;
    points_per_line = 210;
}


void lidar::Lidar::setLines(double num_lines_in){
    num_lines=num_lines_in;
}


void lidar::Lidar::setVerticalAngle(double vertical_angle_in){
    vertical_angle = vertical_angle_in;
}


void lidar::Lidar::setVerticalResolution(double vertical_angle_resolution_in){
    vertical_angle_resolution = vertical_angle_resolution_in;
}


void lidar::Lidar::setHorizontalAngle(double horizontal_angle_in){
    horizontal_angle = horizontal_angle_in;
}


void lidar::Lidar::setHorizontalResolution(double horizontal_angle_resolution_in){
    horizontal_angle_resolution = horizontal_angle_resolution_in; 
}


void lidar::Lidar::setScanPeriod(double scan_period_in){
    scan_period = scan_period_in;
}


void lidar::Lidar::setMaxDistance(double max_distance_in){
	max_distance = max_distance_in;
}

void lidar::Lidar::setMinDistance(double min_distance_in){
	min_distance = min_distance_in;
}