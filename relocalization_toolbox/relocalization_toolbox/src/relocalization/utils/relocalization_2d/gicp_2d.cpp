/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/gicp_2d.h>

namespace gicp_2d
{
    gicp_2d::gicp_2d(const float &max_correspondence_distance,
                     const float &transformation_epsilon,
                     const float &euclidean_fitness_epsilon,
                     const int &max_iterations,
                     const int &map_occupied_threshold,
                     const bool &enable_visualization)
        : max_correspondence_distance_(max_correspondence_distance), transformation_epsilon_(transformation_epsilon),
          euclidean_fitness_epsilon_(euclidean_fitness_epsilon), max_iterations_(max_iterations),
          map_occupied_threshold_(map_occupied_threshold), enable_visualization_(enable_visualization)
    {
        scan_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        map_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

        scan_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("relocalization_scan_cloud", 1);
        map_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("relocalization_map_cloud", 1);

        return;
    };

    void gicp_2d::set_scan(const sensor_msgs::LaserScan &scan,
                           const int &lidar_sampling_step)
    {
        // scan to cloud
        scan_cloud_->clear();
        size_t size = scan.ranges.size();
        float endpoint_dist_min = INFINITY;
        size_t beam_counter = 0;
        bool has_last_point = false;
        pcl::PointXYZ point_last;

        for (size_t i = 0; i < size; i++)
        {
            float range = scan.ranges[i];

            if (range < scan.range_max && range > scan.range_min && isfinite(range))
            {
                float angle = scan.angle_min + i * scan.angle_increment;

                pcl::PointXYZ point;
                point.x = range * cos(angle);
                point.y = range * sin(angle);
                point.z = 0.0;

                if (!has_last_point)
                {
                    point_last = point;
                    has_last_point = true;
                    beam_counter = 1;

                    scan_cloud_->points.push_back(point);

                    continue;
                }

                beam_counter++;

                if (beam_counter >= lidar_sampling_step)
                {
                    endpoint_dist_min = min(endpoint_dist_min,
                                            calculate_distance(point, point_last));
                    beam_counter = 1;
                    point_last = point;
                }

                scan_cloud_->points.push_back(point);
            }
        }

        scan_cloud_->width = scan_cloud_->points.size();
        scan_cloud_->height = 1;
        scan_cloud_->is_dense = true;

        // downsampling
        endpoint_dist_min = max(endpoint_dist_min, map_resolution_ * map_sampling_ratio_);

        if (!scan_cloud_->empty())
        {
            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(scan_cloud_);
            voxel.setLeafSize(endpoint_dist_min, endpoint_dist_min, endpoint_dist_min);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxel.filter(*filtered);

            scan_cloud_.swap(filtered);

            scan_cloud_->width = (uint32_t)scan_cloud_->points.size();
            scan_cloud_->height = 1;
            scan_cloud_->is_dense = true;
        }

        // scan info
        scan_max_range_ = scan.range_max;

        // visualization
        if (enable_visualization_ && !scan_cloud_->empty())
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*scan_cloud_, msg);

            msg.header.frame_id = scan.header.frame_id;
            msg.header.stamp = ros::Time::now();

            scan_cloud_pub_.publish(msg);
        }
    }

    void gicp_2d::set_map(const nav_msgs::OccupancyGrid &map,
                          const int &map_sampling_ratio)
    {
        // map to cloud
        map_cloud_->clear();

        const int width = (int)map.info.width;
        const int height = (int)map.info.height;
        const float resolution = map.info.resolution;
        const float ox = map.info.origin.position.x;
        const float oy = map.info.origin.position.y;

        if (width <= 0 || height <= 0)
        {
            map_cloud_->width = 0;
            map_cloud_->height = 1;
            map_cloud_->is_dense = true;

            return;
        }

        vector<int> row_count(height, 0);

#pragma omp parallel for schedule(static)
        for (int y = 0; y < height; y++)
        {
            int c = 0;
            const int base = y * width;
            for (int x = 0; x < width; x++)
            {
                const int idx = base + x;
                if (map.data[idx] >= map_occupied_threshold_)
                {
                    c++;
                }
            }
            row_count[y] = c;
        }

        vector<int> row_offset(height + 1, 0);

        for (int y = 0; y < height; y++)
        {
            row_offset[y + 1] = row_offset[y] + row_count[y];
        }

        const int total_occ = row_offset[height];

        if (total_occ <= 0)
        {
            map_cloud_->width = 0;
            map_cloud_->height = 1;
            map_cloud_->is_dense = true;
            return;
        }

        map_cloud_->points.resize((size_t)total_occ);

#pragma omp parallel for schedule(static)
        for (int y = 0; y < height; y++)
        {
            int write = row_offset[y];
            const int base = y * width;
            const float wy = oy + ((float)y + 0.5f) * resolution;

            for (int x = 0; x < width; x++)
            {
                const int idx = base + x;
                if (map.data[idx] >= map_occupied_threshold_)
                {
                    const float wx = ox + ((float)x + 0.5f) * resolution;

                    pcl::PointXYZ p;
                    p.x = wx;
                    p.y = wy;
                    p.z = 0.0f;

                    map_cloud_->points[(size_t)write] = p;
                    write++;
                }
            }
        }

        map_cloud_->width = (uint32_t)map_cloud_->points.size();
        map_cloud_->height = 1;
        map_cloud_->is_dense = true;

        // downsampling
        if (map_sampling_ratio > 1 && !map_cloud_->empty())
        {
            const float leaf_size = (float)map_sampling_ratio * resolution;

            pcl::VoxelGrid<pcl::PointXYZ> voxel;
            voxel.setInputCloud(map_cloud_);
            voxel.setLeafSize(leaf_size, leaf_size, leaf_size);

            pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
            voxel.filter(*filtered);

            map_cloud_.swap(filtered);

            map_cloud_->width = (uint32_t)map_cloud_->points.size();
            map_cloud_->height = 1;
            map_cloud_->is_dense = true;
        }

        // kdtree for local map
        if (!map_cloud_->empty())
        {
            map_kdtree_->setInputCloud(map_cloud_);
        }

        // map info
        map_sampling_ratio_ = map_sampling_ratio;
        map_resolution_ = map.info.resolution;

        // visualization
        if (enable_visualization_ && !map_cloud_->empty())
        {
            sensor_msgs::PointCloud2 msg;
            pcl::toROSMsg(*map_cloud_, msg);

            msg.header.frame_id = map.header.frame_id;
            msg.header.stamp = ros::Time::now();

            map_cloud_pub_.publish(msg);
        }
    }

    void gicp_2d::build_local_map(const float &center_x,
                                  const float &center_y,
                                  const float &radius,
                                  pcl::PointCloud<pcl::PointXYZ>::Ptr &local_map)
    {
        pcl::PointXYZ query;
        query.x = center_x;
        query.y = center_y;
        query.z = 0.0f;

        vector<int> indices;
        vector<float> sqr_dists;

        if (map_kdtree_->radiusSearch(query, radius, indices, sqr_dists) > 0)
        {
            local_map->points.reserve(indices.size());

            for (const int &id : indices)
            {
                local_map->points.push_back(map_cloud_->points[id]);
            }

            local_map->width = local_map->points.size();
            local_map->height = 1;
            local_map->is_dense = true;
        }
    }

    geometry_msgs::TransformStamped gicp_2d::match(const geometry_msgs::TransformStamped &predict)
    {
        geometry_msgs::TransformStamped ret;

        pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        build_local_map(predict.transform.translation.x,
                        predict.transform.translation.y,
                        scan_max_range_ + 4.0f,
                        local_map_cloud);

        if (local_map_cloud->empty())
        {
            return predict;
        }

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(scan_cloud_);
        icp.setInputTarget(local_map_cloud);

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
        ret.child_frame_id = predict.child_frame_id;
        ret.transform.translation.x = tf(0, 3);
        ret.transform.translation.y = tf(1, 3);
        ret.transform.translation.z = 0.0;

        tf2::Matrix3x3 rot(
            tf(0, 0), tf(0, 1), tf(0, 2),
            tf(1, 0), tf(1, 1), tf(1, 2),
            tf(2, 0), tf(2, 1), tf(2, 2));
        tf2::Quaternion q;
        rot.getRotation(q);
        q.normalize();
        ret.transform.rotation = tf2::toMsg(q);

        return ret;
    }
};