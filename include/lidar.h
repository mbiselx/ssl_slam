// Author of SSL_SLAM: Wang Han 
// Email wh200720041@gmail.com
// Homepage https://wanghan.pro
#ifndef _LIDAR_H_
#define _LIDAR_H_

//define lidar parameter

namespace lidar{

class Lidar
{
    public:
        Lidar();

        void setScanPeriod(double scan_period_in);
        void setLines(double num_lines_in);
        void setVerticalAngle(double vertical_angle_in);
        void setVerticalResolution(double vertical_angle_resolution_in);
        void setHorizontalAngle(double horizontal_angle_in);
        void setHorizontalResolution(double horizontal_angle_resolution_in);
        //by default is 100. pls do not change
        void setMaxDistance(double max_distance_in);
        void setMinDistance(double min_distance_in);

    	double max_distance;
        double min_distance;

        double scan_period;

        int num_lines;
        int points_per_line;

        double horizontal_angle;
        double horizontal_angle_resolution;
        double min_horizontal_angle;
        double max_horizontal_angle;

        double vertical_angle;
        double vertical_angle_resolution;
        double min_vertical_angle;
        double max_vertical_angle;
};


}

#endif // _LIDAR_H_
