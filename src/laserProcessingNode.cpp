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
#include <ros/console.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "lidar.h"
#include "laserProcessingClass.h"


LaserProcessingClass laserProcessing;
std::mutex mutex_lock;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;
lidar::Lidar lidar_param;

ros::Publisher pubEdgePoints;
ros::Publisher pubSurfPoints;
ros::Publisher pubLaserCloudFiltered;

void velodyneHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void laser_processing(){
    double total_time = 0;
    int total_frame = 0;

    std::chrono::time_point<std::chrono::system_clock> operation_start;
    std::chrono::duration<float> elapsed_seconds; 

    std_msgs::Header pointcloud_header;
    sensor_msgs::PointCloud2 surfPointsMsg;
    sensor_msgs::PointCloud2 edgePointsMsg;
    sensor_msgs::PointCloud2 laserCloudFilteredMsg;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge(new pcl::PointCloud<pcl::PointXYZI>());          
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());  

    while(ros::ok()){

        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        // make sure there's something to do
        if(pointCloudBuf.empty())
            continue;

        //read data
        mutex_lock.lock();
        pcl::fromROSMsg(*pointCloudBuf.front(), *pointcloud_in);
        pointcloud_header = pointCloudBuf.front()->header;
        pointCloudBuf.pop();
        mutex_lock.unlock();

        // feature extraction operation
        operation_start = std::chrono::system_clock::now();
        laserProcessing.featureExtraction(pointcloud_in, pointcloud_edge, pointcloud_surf);
        elapsed_seconds = std::chrono::system_clock::now() - operation_start;

        // statistics for nerds
        total_time+=(elapsed_seconds.count() * 1000);
        total_frame++;

        if(total_frame%20==0){
            ROS_DEBUG("[laser processing]: average laser processing time %f ms", total_time/total_frame);
            total_time = 0;
            total_frame = 0;
        }

        // output the processed pointclouds
        pointcloud_filtered->clear();
        *pointcloud_filtered+=*pointcloud_edge;
        *pointcloud_filtered+=*pointcloud_surf;
        pcl::toROSMsg(*pointcloud_filtered, laserCloudFilteredMsg);
        laserCloudFilteredMsg.header = pointcloud_header;
        pubLaserCloudFiltered.publish(laserCloudFilteredMsg);

        pcl::toROSMsg(*pointcloud_edge, edgePointsMsg);
        edgePointsMsg.header = pointcloud_header;
        pubEdgePoints.publish(edgePointsMsg);

        pcl::toROSMsg(*pointcloud_surf, surfPointsMsg);
        surfPointsMsg.header = pointcloud_header;
        pubSurfPoints.publish(surfPointsMsg);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

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

    laserProcessing.init(lidar_param);

    ros::Subscriber subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 100, velodyneHandler);
    pubLaserCloudFiltered = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud/filtered", 100);
    pubEdgePoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud/edge", 100);
    pubSurfPoints = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud/surf", 100); 

    std::thread laser_processing_process{laser_processing};

    ROS_INFO("[laser processing]: laser processing inited");

    ros::spin();

    return 0;
}
