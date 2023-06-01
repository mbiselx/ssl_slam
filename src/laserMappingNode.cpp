// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

//pcl lib
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laserMappingClass.h"
#include "lidar.h"


LaserMappingClass laserMapping;
lidar::Lidar lidar_param;
std::mutex mutex_lock;
std::queue<nav_msgs::OdometryConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
Eigen::Isometry3d last_pose = Eigen::Isometry3d::Identity();
std::string map_frame = "map";
std::string robot_frame = "base_link";

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    mutex_lock.lock();
    odometryBuf.push(msg);
    mutex_lock.unlock();
}

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void laser_mapping(){

    double displacement, angular_change;
    sensor_msgs::PointCloud2 pcl_s, pcl_m; 

    tf::StampedTransform transform;    
    tf::TransformListener listener;     

    Eigen::Isometry3d current_pose, delta_transform;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_map(new pcl::PointCloud<pcl::PointXYZI>()); 

    while(ros::ok()){

        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);


        // make sure there's something to do
        if(odometryBuf.empty() || pointCloudBuf.empty())
            continue;

        // read data, making sure the time is aligned
        mutex_lock.lock();
        if(pointCloudBuf.front()->header.stamp.toSec()<odometryBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
            ROS_WARN("[laser mapping]: time stamp unaligned error and pointcloud discarded, pls check your data"); 
            pointCloudBuf.pop();
            mutex_lock.unlock();
            continue;              
        }

        if(odometryBuf.front()->header.stamp.toSec() < pointCloudBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
            odometryBuf.pop();
            ROS_WARN("[laser mapping]: time stamp unaligned with path final, pls check your data");
            mutex_lock.unlock();
            continue;  
        }

        pcl_s = *pointCloudBuf.front(); 
        pointCloudBuf.pop();
        current_pose = Eigen::Isometry3d::Identity();
        current_pose.rotate(Eigen::Quaterniond(
            odometryBuf.front()->pose.pose.orientation.w,
            odometryBuf.front()->pose.pose.orientation.x,
            odometryBuf.front()->pose.pose.orientation.y,
            odometryBuf.front()->pose.pose.orientation.z
            ));  
        current_pose.pretranslate(Eigen::Vector3d(
            odometryBuf.front()->pose.pose.position.x,
            odometryBuf.front()->pose.pose.position.y,
            odometryBuf.front()->pose.pose.position.z
            ));
        odometryBuf.pop();
        mutex_lock.unlock();
        
        // find displacement between this and last pointcloud
        delta_transform = last_pose.inverse() * current_pose;
        displacement = delta_transform.translation().squaredNorm();
        angular_change = delta_transform.linear().eulerAngles(2,1,0)[0] * 180 / M_PI;

        if(angular_change > 90) 
            angular_change = fabs(180 - angular_change);
        
        // discard points if they are not novel enough
        if(displacement < 0.3 && angular_change < 20)
            continue; 

        // transform pointcloud to the correct frame
        pcl::fromROSMsg(pcl_s, *pointcloud_in);
        if (!pcl_ros::transformPointCloud(robot_frame, *pointcloud_in, *pointcloud_map, listener)) {
            ROS_WARN("[laser mapping]: transform '%s'->'%s' failed -- cannot update map", pcl_s.header.frame_id.c_str(), robot_frame.c_str() );
            continue;
        }

        // update the internal map
        ROS_INFO("[laser mapping]: updating map : (%.2fm, %.2fdeg) ", displacement, angular_change);
        laserMapping.updateCurrentPointsToMap(pointcloud_map, current_pose);
        last_pose = current_pose;   

    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    double map_resolution = 0.4;

    nh.getParam("/max_dis", lidar_param.max_distance);
    nh.getParam("/min_dis", lidar_param.min_distance);

    nh.getParam("/scan_period", lidar_param.scan_period); 
    
    nh.getParam("/vertical_angle", lidar_param.vertical_angle); 
    nh.getParam("/vertical_angle_resolution", lidar_param.vertical_angle_resolution); 
    nh.getParam("/min_vertical_angle", lidar_param.min_vertical_angle); 
    nh.getParam("/max_vertical_angle", lidar_param.max_vertical_angle); 
    nh.getParam("/horizontal_angle_resolution", lidar_param.horizontal_angle_resolution); 
    nh.getParam("/horizontal_angle", lidar_param.horizontal_angle); 
    nh.getParam("/min_horizontal_angle", lidar_param.min_horizontal_angle); 
    nh.getParam("/max_horizontal_angle", lidar_param.max_horizontal_angle); 

    nh.getParam("/map_resolution", map_resolution);
    nh.getParam("/map_frame", map_frame);
    nh.getParam("/robot_frame", robot_frame);
    
    laserMapping.init(map_resolution);
    last_pose.translation().x() = 10;
    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("laser_cloud/filtered", 100, velodyneHandler);
    ros::Subscriber subOdometry = nh.subscribe<nav_msgs::Odometry>("/odom", 100, odomCallback);

    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/map", 100);
    std::thread laser_mapping_process{laser_mapping};

    int map_seq = 0;
    sensor_msgs::PointCloud2 pointcloud_map; 
    while (ros::ok()) {
        if (map_pub.getNumSubscribers()) {
            // publish full map periodically, if anyone is listening
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZI>);
            pcl_map = laserMapping.getMap();
            pcl::toROSMsg(*pcl_map, pointcloud_map);
            pointcloud_map.header.stamp = ros::Time::now();
            pointcloud_map.header.frame_id = map_frame;
            pointcloud_map.header.seq = map_seq++;
            map_pub.publish(pointcloud_map); 
            ros::Rate(2).sleep();
        }
        else {
            ros::Rate(10).sleep();
        }
        ros::spinOnce();
    }

    return 0;
}
