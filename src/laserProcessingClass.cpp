// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#include "laserProcessingClass.h"

void LaserProcessingClass::init(lidar::Lidar lidar_param_in){
    
    lidar_param = lidar_param_in;

}

void LaserProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    // make sure the pointclouds are clear
    pc_out_edge->clear();
    pc_out_surf->clear();

    // prepare sectors
    int m =  (lidar_param.max_horizontal_angle - lidar_param.min_horizontal_angle)/lidar_param.horizontal_angle_resolution/2; 
    int n =  (lidar_param.max_vertical_angle - lidar_param.min_vertical_angle)/lidar_param.vertical_angle_resolution/2; 
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> laserCloudScans;
    for (int h = 0; h < m; h++){
        for (int v = 0; v < n; v++){
            laserCloudScans.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>()));
        }
    }

    // fill in sectors 
    for (auto point = pc_in->points.begin(); point != pc_in->points.end(); point++)
    {
        // calulate the distance to this point
        double distance = sqrt(point->x*point->x + point->y*point->y + point->z*point->z);
        
        // skip points outside of the regions of interest
        if(isnan(distance) || distance<lidar_param.min_distance || distance > lidar_param.max_distance )
            continue;

        // bin points into sectors
        double azimuth = (double) atan2(point->y, point->x) * 180 / M_PI;
        double elevation = asin(point->z/distance) * 180 / M_PI;


        int h_bin = (int) ((azimuth - lidar_param.min_horizontal_angle) * m / (lidar_param.max_horizontal_angle - lidar_param.min_horizontal_angle));
        if (h_bin < 0 || h_bin > m) {
            ROS_ERROR_STREAM("BAD HBIN :" << h_bin << " / " << m << " ("  << azimuth << " / " << lidar_param.min_horizontal_angle << " / " << lidar_param.horizontal_angle_resolution << ")");
            continue;
        }
        int v_bin = (int) ((elevation - lidar_param.min_vertical_angle) * n / (lidar_param.max_vertical_angle - lidar_param.min_vertical_angle));
        if (v_bin < 0 || v_bin > n) {
            ROS_ERROR_STREAM("BAD VBIN :" << v_bin << " / " << n << " (" << elevation << " / " << lidar_param.min_vertical_angle << " / " << lidar_param.vertical_angle_resolution << ")");
            continue;
        }
        // forgive smaller infractions
        if (h_bin == m)
            h_bin--;
        if (v_bin == n)
            v_bin--;         

        // if OK, add the point to cloud
        laserCloudScans[n*h_bin + v_bin]->push_back(*point);
    }

    ROS_DEBUG_ONCE("[laser processing]: total sectors in array %ld", laserCloudScans.size());

    for(auto sector_itr = laserCloudScans.begin(); sector_itr != laserCloudScans.end(); sector_itr++) {
        std::vector<Double2d> cloudCurvature; 
        pcl::PointCloud<pcl::PointXYZI>::Ptr sector = *sector_itr;
        for(int j = 5; j < (int)sector->points.size() - 5; j++){
            double diffX = sector->points[j - 5].x + sector->points[j - 4].x + sector->points[j - 3].x + sector->points[j - 2].x + sector->points[j - 1].x - 10 * sector->points[j].x + sector->points[j + 1].x + sector->points[j + 2].x + sector->points[j + 3].x + sector->points[j + 4].x + sector->points[j + 5].x;
            double diffY = sector->points[j - 5].y + sector->points[j - 4].y + sector->points[j - 3].y + sector->points[j - 2].y + sector->points[j - 1].y - 10 * sector->points[j].y + sector->points[j + 1].y + sector->points[j + 2].y + sector->points[j + 3].y + sector->points[j + 4].y + sector->points[j + 5].y;
            double diffZ = sector->points[j - 5].z + sector->points[j - 4].z + sector->points[j - 3].z + sector->points[j - 2].z + sector->points[j - 1].z - 10 * sector->points[j].z + sector->points[j + 1].z + sector->points[j + 2].z + sector->points[j + 3].z + sector->points[j + 4].z + sector->points[j + 5].z;
            Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
            cloudCurvature.push_back(distance);

        }
        featureExtractionFromSector(sector, cloudCurvature, pc_out_edge, pc_out_surf);
    }

    // for(uint8_t i = 0; i < laserCloudScans.size(); i++){
    //     std::vector<Double2d> cloudCurvature; 
    //     for(int j = 5; j < (int)laserCloudScans[i]->points.size() - 5; j++){
    //         double diffX = laserCloudScans[i]->points[j - 5].x + laserCloudScans[i]->points[j - 4].x + laserCloudScans[i]->points[j - 3].x + laserCloudScans[i]->points[j - 2].x + laserCloudScans[i]->points[j - 1].x - 10 * laserCloudScans[i]->points[j].x + laserCloudScans[i]->points[j + 1].x + laserCloudScans[i]->points[j + 2].x + laserCloudScans[i]->points[j + 3].x + laserCloudScans[i]->points[j + 4].x + laserCloudScans[i]->points[j + 5].x;
    //         double diffY = laserCloudScans[i]->points[j - 5].y + laserCloudScans[i]->points[j - 4].y + laserCloudScans[i]->points[j - 3].y + laserCloudScans[i]->points[j - 2].y + laserCloudScans[i]->points[j - 1].y - 10 * laserCloudScans[i]->points[j].y + laserCloudScans[i]->points[j + 1].y + laserCloudScans[i]->points[j + 2].y + laserCloudScans[i]->points[j + 3].y + laserCloudScans[i]->points[j + 4].y + laserCloudScans[i]->points[j + 5].y;
    //         double diffZ = laserCloudScans[i]->points[j - 5].z + laserCloudScans[i]->points[j - 4].z + laserCloudScans[i]->points[j - 3].z + laserCloudScans[i]->points[j - 2].z + laserCloudScans[i]->points[j - 1].z - 10 * laserCloudScans[i]->points[j].z + laserCloudScans[i]->points[j + 1].z + laserCloudScans[i]->points[j + 2].z + laserCloudScans[i]->points[j + 3].z + laserCloudScans[i]->points[j + 4].z + laserCloudScans[i]->points[j + 5].z;
    //         Double2d distance(j, diffX * diffX + diffY * diffY + diffZ * diffZ);
    //         cloudCurvature.push_back(distance);
    //     }
    //     featureExtractionFromSector(laserCloudScans[i], cloudCurvature, pc_out_edge, pc_out_surf);
    // }

}


void LaserProcessingClass::featureExtractionFromSector(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, std::vector<Double2d>& cloudCurvature, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_edge, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out_surf){

    std::sort(cloudCurvature.begin(), cloudCurvature.end(), [](const Double2d & a, const Double2d & b)
    { 
        return a.value < b.value; 
    });


    int largestPickedNum = 0;
    std::vector<int> picked_points;
    int point_info_count =0;
    for (int i = cloudCurvature.size()-1; i >= 0; i--)
    {
        int ind = cloudCurvature[i].id; 
        if(std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end()){
            if(cloudCurvature[i].value <= 0.1){
                break;
            }
            
            largestPickedNum++;
            picked_points.push_back(ind);
            
            if (largestPickedNum <= 10){
                pc_out_edge->push_back(pc_in->points[ind]);
                point_info_count++;
            }else{
                break;
            }

            for(int k=1;k<=5;k++){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k - 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k - 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }
            for(int k=-1;k>=-5;k--){
                double diffX = pc_in->points[ind + k].x - pc_in->points[ind + k + 1].x;
                double diffY = pc_in->points[ind + k].y - pc_in->points[ind + k + 1].y;
                double diffZ = pc_in->points[ind + k].z - pc_in->points[ind + k + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05){
                    break;
                }
                picked_points.push_back(ind+k);
            }

        }
    }
    
    for (int i = 0; i <= (int)cloudCurvature.size()-1; i++)
    {
        int ind = cloudCurvature[i].id; 
        if( std::find(picked_points.begin(), picked_points.end(), ind)==picked_points.end())
        {
            pc_out_surf->push_back(pc_in->points[ind]);
        }
    }
    


}
LaserProcessingClass::LaserProcessingClass(){
    
}

Double2d::Double2d(int id_in, double value_in){
    id = id_in;
    value =value_in;
};

PointsInfo::PointsInfo(int layer_in, double time_in){
    layer = layer_in;
    time = time_in;
};
