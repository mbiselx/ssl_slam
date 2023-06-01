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
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

//pcl lib
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/impl/transforms.hpp>

//local lib
#include "lidar.h"
#include "odomEstimationClass.h"


std::mutex mutex_lock;

std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudEdgeBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudSurfBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> pointCloudBuf;

lidar::Lidar lidar_param;
std::string map_frame = "map";
std::string robot_frame = "base_link";

ros::Publisher pubLaserOdometry;
OdomEstimationClass odomEstimation; 

void velodyneSurfHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudSurfBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}
void velodyneEdgeHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
    mutex_lock.lock();
    pointCloudEdgeBuf.push(laserCloudMsg);
    mutex_lock.unlock();
}

void odom_estimation(){
    bool is_odom_inited = false;
    double total_time = 0;
    int total_frame = 0;

    std::chrono::time_point<std::chrono::system_clock> operation_start;
    std::chrono::duration<float> elapsed_seconds; 

    std_msgs::Header pointcloud_header;
    tf::TransformListener listener;
    tf::TransformBroadcaster br;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_surf_r(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_in(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_edge_r(new pcl::PointCloud<pcl::PointXYZI>());

    while(ros::ok()){

        //sleep 2 ms every time
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);

        // make sure there's something to do
        if(pointCloudEdgeBuf.empty() || pointCloudSurfBuf.empty())
            continue;

        // read data, making sure the time is aligned
        mutex_lock.lock();
        if(pointCloudSurfBuf.front()->header.stamp.toSec() < pointCloudEdgeBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
            pointCloudSurfBuf.pop();
            ROS_WARN("[odom estimation]: time stamp unaligned with extra point cloud, pls check your data");
            mutex_lock.unlock();
            continue;  
        }

        if(pointCloudEdgeBuf.front()->header.stamp.toSec() < pointCloudSurfBuf.front()->header.stamp.toSec()-0.5*lidar_param.scan_period){
            pointCloudEdgeBuf.pop();
            ROS_WARN("[odom estimation]: time stamp unaligned with extra point cloud, pls check your data");
            mutex_lock.unlock();
            continue;  
        }

        pointcloud_header = pointCloudSurfBuf.front()->header;
        pcl::fromROSMsg(*pointCloudSurfBuf.front(), *pointcloud_surf_in);
        pointCloudSurfBuf.pop();
        pcl::fromROSMsg(*pointCloudEdgeBuf.front(), *pointcloud_edge_in);
        pointCloudEdgeBuf.pop();
        mutex_lock.unlock();

        if (!pcl_ros::transformPointCloud(robot_frame, *pointcloud_surf_in, *pointcloud_surf_r, listener) || 
            !pcl_ros::transformPointCloud(robot_frame, *pointcloud_edge_in, *pointcloud_edge_r, listener)) {
            ROS_WARN("[odom estimation]: transform '%s'->'%s' failed -- cannot put pointcloud in robot frame",
                pointcloud_edge_in->header.frame_id.c_str(), robot_frame.c_str());
            continue;
        }

        if(is_odom_inited) {
            // update map operation
            operation_start = std::chrono::system_clock::now();
            odomEstimation.updatePointsToMap(pointcloud_edge_r, pointcloud_surf_r);
            // odomEstimation.updatePointsToMap(pointcloud_edge_in, pointcloud_surf_in);
            elapsed_seconds = std::chrono::system_clock::now() - operation_start;

            // statistics for nerds
            total_time+=elapsed_seconds.count() * 1000;
            total_frame++;

            if(total_frame%20==0) {
                ROS_DEBUG("[odom estimation]: average odom estimation time %f ms", total_time/total_frame);
                total_time = 0;
                total_frame = 0;            
            }
        }
        else { // initialize odometry map estimator
            odomEstimation.initMapWithPoints(pointcloud_edge_r, pointcloud_surf_r);
            // odomEstimation.initMapWithPoints(pointcloud_edge_in, pointcloud_surf_in);
            is_odom_inited = true;
            ROS_INFO("[odom estimation]: odom inited");
        }

        // create odometry (which is in the robot frame, by default)
        Eigen::Vector3d t_current = odomEstimation.odom.translation();
        Eigen::Quaterniond q_current(odomEstimation.odom.rotation());
        q_current.normalize();

        geometry_msgs::PoseStamped ps;
        ps.header = pointcloud_header;
        ps.header.frame_id = robot_frame;
        ps.pose.position.x = t_current.x();
        ps.pose.position.y = t_current.y();
        ps.pose.position.z = t_current.z();
        ps.pose.orientation.x = q_current.x();
        ps.pose.orientation.y = q_current.y();
        ps.pose.orientation.z = q_current.z();
        ps.pose.orientation.w = q_current.w();

        // broadcast new transform
        tf::StampedTransform transform(
            tf::Transform( 
                tf::Quaternion(
                    ps.pose.orientation.x,
                    ps.pose.orientation.y,
                    ps.pose.orientation.z,
                    ps.pose.orientation.w
                ),
                tf::Vector3(
                    ps.pose.position.x, 
                    ps.pose.position.y, 
                    ps.pose.position.z
                )
            ),
            ps.header.stamp, 
            map_frame, 
            robot_frame
        );
        br.sendTransform(transform);

        // publish odometry
        nav_msgs::Odometry laserOdometry;
        laserOdometry.header.frame_id = map_frame; 
        laserOdometry.child_frame_id = robot_frame; 
        laserOdometry.header.stamp = ps.header.stamp;
        laserOdometry.pose.pose = ps.pose;
        pubLaserOdometry.publish(laserOdometry);

    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    double map_resolution = 0.4;
    double optimization_iterations = 2;

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
    nh.getParam("/optimization_iterations", optimization_iterations);
    nh.getParam("/map_frame", map_frame);
    nh.getParam("/robot_frame", robot_frame);

    odomEstimation.init(lidar_param, map_resolution, optimization_iterations);

    ros::Subscriber subEdgeLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud/edge", 100, velodyneEdgeHandler);
    ros::Subscriber subSurfLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud/surf", 100, velodyneSurfHandler);
    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/odom", 100);
    ros::Publisher map_pub = nh.advertise<sensor_msgs::PointCloud2>("/internal_map", 100);
    std::thread odom_estimation_process{odom_estimation};

    int map_seq = 0;
    sensor_msgs::PointCloud2 pointcloud_map; 
    while (ros::ok()) {
        if (map_pub.getNumSubscribers()) {
            // publish full map periodically, if anyone is listening
            pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZI>);
            odomEstimation.getMap(pcl_map);
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
