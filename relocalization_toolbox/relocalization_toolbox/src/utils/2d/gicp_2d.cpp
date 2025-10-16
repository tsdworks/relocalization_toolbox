/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <utils/2d/gicp_2d.h>

namespace gicp_2d
{
    gicp_2d::gicp_2d(const float &max_correspondence_distance,
                     const float &transformation_epsilon,
                     const float &euclidean_fitness_epsilon,
                     const int &max_iterations,
                     const int &map_occupied_threshold)
        : max_correspondence_distance_(max_correspondence_distance), transformation_epsilon_(transformation_epsilon),
          euclidean_fitness_epsilon_(euclidean_fitness_epsilon), max_iterations_(max_iterations),
          map_occupied_threshold_(map_occupied_threshold)
    {
        scan_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        map_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

        return;
    };

    void gicp_2d::set_scan(const sensor_msgs::LaserScan &scan)
    {
        // scan to cloud
        scan_cloud_->clear();

        for (size_t i = 0; i < scan.ranges.size(); i++)
        {
            float range = scan.ranges[i];

            if (range < scan.range_max && range > scan.range_min && isfinite(range))
            {
                float angle = scan.angle_min + i * scan.angle_increment;

                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;

                scan_cloud_->points.push_back(point);
            }
        }

        scan_cloud_->width = scan_cloud_->points.size();
        scan_cloud_->height = 1;
        scan_cloud_->is_dense = true;
    }

    void gicp_2d::set_map(const nav_msgs::OccupancyGrid &map)
    {
        // map to cloud
        map_cloud_->clear();

        float resolution = map.info.resolution;

        for (unsigned int y = 0; y < map.info.height; y++)
        {
            for (unsigned int x = 0; x < map.info.width; x++)
            {
                int idx = y * map.info.width + x;

                if (map.data[idx] >= map_occupied_threshold_)
                {
                    float wx = map.info.origin.position.x + x * resolution;
                    float wy = map.info.origin.position.y + y * resolution;

                    pcl::PointXYZ point;
                    point.x = wx;
                    point.y = wy;
                    point.z = 0.0;

                    map_cloud_->points.push_back(point);
                }
            }
        }

        map_cloud_->width = map_cloud_->points.size();
        map_cloud_->height = 1;
        map_cloud_->is_dense = true;
    }

    geometry_msgs::TransformStamped gicp_2d::match(const geometry_msgs::TransformStamped &predict)
    {
        geometry_msgs::TransformStamped ret;

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(scan_cloud_);
        icp.setInputTarget(map_cloud_);

        icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
        icp.setTransformationEpsilon(transformation_epsilon_);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
        icp.setMaximumIterations(max_iterations_);

        Eigen::Isometry3d init = tf2::transformToEigen(predict);
        Eigen::Matrix4f init_guess = init.matrix().cast<float>();

        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final, init_guess);

        Eigen::Matrix4f tf = icp.getFinalTransformation();

        ret.header.stamp = predict.header.stamp;
        ret.header.frame_id = predict.header.frame_id;
        ret.child_frame_id = predict.header.frame_id;
        ret.transform.translation.x = tf(0, 3);
        ret.transform.translation.y = tf(1, 3);
        ret.transform.translation.z = 0.0;

        tf2::Matrix3x3 rot(
            tf(0, 0), tf(0, 1), tf(0, 2),
            tf(1, 0), tf(1, 1), tf(1, 2),
            tf(2, 0), tf(2, 1), tf(2, 2));
        double roll, pitch, yaw;
        rot.getRPY(roll, pitch, yaw);
        tf2::Quaternion q;
        rot.getRotation(q);
        q.normalize();
        ret.transform.rotation = tf2::toMsg(q);

        return ret;
    }
};