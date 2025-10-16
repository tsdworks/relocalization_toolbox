/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <utils/2d/likelihood_field_2d.h>

namespace likelihood_field_2d
{
    likelihood_field_2d::likelihood_field_2d(const float &sigma_hit, const float &z_hit, const float &z_rand, const int &map_occupied_threshold)
        : sigma_hit_(sigma_hit), z_hit_(z_hit), z_rand_(z_rand), map_occupied_threshold_(map_occupied_threshold)
    {
        debug_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>(
            "transformed_beam_points", 1);

        return;
    };

    float likelihood_field_2d::gaussian_prob(float dist)
    {
        return z_hit_ * exp(-(dist * dist) / (2.0f * sigma_hit_ * sigma_hit_)) + z_rand_;
    }

    tuple<float, float, float> likelihood_field_2d::get_likelihood_field_score(
        const geometry_msgs::TransformStamped &tf_map_to_lidar,
        const sensor_msgs::LaserScan &scan,
        const nav_msgs::OccupancyGrid &map,
        const vector<float> &obs_dist_table,
        const int &lidar_sampling_step,
        const bool &enable_visualization)
    {
        vector<pair<float, float>> loglik_dist_pairs;
        vector<tuple<float, float>> beam_points;

        float angle = scan.angle_min;
        float total_dist = 0;
        int valid_beam_count = 0;

        for (size_t i = 0; i < scan.ranges.size(); i += lidar_sampling_step, angle += scan.angle_increment * lidar_sampling_step)
        {
            float r = scan.ranges[i];

            if (r < scan.range_min || r > scan.range_max)
            {
                continue;
            }

            geometry_msgs::PointStamped p_lidar, p_map;
            p_lidar.header.frame_id = scan.header.frame_id;
            p_lidar.point.x = r * cos(angle);
            p_lidar.point.y = r * sin(angle);
            tf2::doTransform(p_lidar, p_map, tf_map_to_lidar);

            geometry_msgs::Point32 pt;
            pt.x = p_map.point.x;
            pt.y = p_map.point.y;
            beam_points.emplace_back(pt.x, pt.y);

            int index = position_to_grid_index(pt, map);

            if (index < 0 || index >= static_cast<int>(obs_dist_table.size()))
            {
                continue;
            }

            float dist = obs_dist_table[index];
            float log_p = log(gaussian_prob(dist) + 1e-6f);
            loglik_dist_pairs.emplace_back(log_p, dist);
            total_dist += dist;

            valid_beam_count++;
        }

        if (enable_visualization)
        {
            sensor_msgs::PointCloud2 cloud_msg;
            cloud_msg.header.frame_id = map.header.frame_id;
            cloud_msg.header.stamp = ros::Time::now();
            cloud_msg.height = 1;
            cloud_msg.is_dense = false;
            cloud_msg.width = beam_points.size();

            sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
            modifier.setPointCloud2FieldsByString(1, "xyz");

            sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

            for (const auto &[x, y] : beam_points)
            {
                *iter_x = x;
                *iter_y = y;
                *iter_z = 0.0f;

                ++iter_x;
                ++iter_y;
                ++iter_z;
            }

            debug_cloud_pub_.publish(cloud_msg);
        }

        if (valid_beam_count < 5)
        {
            return {-numeric_limits<float>::infinity(), numeric_limits<float>::infinity(), -numeric_limits<float>::infinity()};
        }

        sort(loglik_dist_pairs.begin(), loglik_dist_pairs.end(),
             [](const pair<float, float> &a, const pair<float, float> &b)
             {
                 return a.first > b.first;
             });

        int n = loglik_dist_pairs.size();

        float scan_fov = abs(scan.angle_max - scan.angle_min);
        float alpha = clamp((float)((scan_fov - M_PI) / M_PI), 0.0f, 1.0f);
        float keep_ratio_1 = (1.0f - alpha) * 0.90f + alpha * 0.80f;
        float keep_ratio_2 = (1.0f - alpha) * 0.85f + alpha * 0.70f;

        int k100 = n;
        int k1 = min(n, max(5, static_cast<int>(keep_ratio_1 * n)));
        int k2 = min(n, max(5, static_cast<int>(keep_ratio_2 * n)));

        auto avg_score = [&](int k)
        {
            float sum = 0.0f;

            for (int i = 0; i < k; i++)
            {
                sum += loglik_dist_pairs[i].first;
            }

            return sum / k;
        };

        vector<pair<float, int>> score_candidates = {
            {avg_score(k100), k100},
            {avg_score(k1), k1},
            {avg_score(k2), k2}};

        sort(score_candidates.begin(), score_candidates.end(), greater<>());
        float avg_log_likelihood = score_candidates[1].first;
        int selected_k = score_candidates[1].second;

        float sum_sq = 0.0f;

        for (int i = 0; i < selected_k; i++)
        {
            float d = loglik_dist_pairs[i].second;

            sum_sq += d * d;
        }

        float mean_dist = total_dist / valid_beam_count;
        float variance = (sum_sq / selected_k) - (mean_dist * mean_dist);
        variance = max(0.0f, variance);
        float consistency_score = exp(-variance / 2.0f);

        return {avg_log_likelihood, mean_dist, consistency_score};
    }

    tuple<float, float, float> likelihood_field_2d::get_likelihood_field_score(const geometry_msgs::TransformStamped &tf_map_to_lidar,
                                                                               const sensor_msgs::LaserScan &scan,
                                                                               const nav_msgs::OccupancyGrid &map,
                                                                               const int &lidar_sampling_step,
                                                                               const bool &enable_visualization)
    {
        float max_range = scan.range_max;
        float radius = max_range + 2.0f;
        vector<float> distance_map = calc_min_dist_to_obs_table(map,
                                                                tf_map_to_lidar.transform.translation.x,
                                                                tf_map_to_lidar.transform.translation.y,
                                                                radius, map_occupied_threshold_);

        return get_likelihood_field_score(tf_map_to_lidar, scan, map, distance_map,
                                          lidar_sampling_step,
                                          enable_visualization);
    }
};