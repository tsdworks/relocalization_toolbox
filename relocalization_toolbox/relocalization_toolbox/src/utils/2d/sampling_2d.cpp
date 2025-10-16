/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <utils/2d/sampling_2d.h>

namespace sampling_2d
{
    sampling_2d::sampling_2d(const float &min_expand_dist, const float &max_expand_dist, const float &min_dist_to_obs,
                             const float &points_min_dist,
                             const int &free_threshold, const int &occupied_threshold)
    {
        random_device_ = make_unique<random_device>();
        random_generator_ = make_unique<mt19937>((*random_device_)());
        gen_random_number_ = make_unique<uniform_real_distribution<float>>(0.0f, 1.0f);

        free_threshold_ = free_threshold;
        occupied_threshold_ = occupied_threshold;

        min_expand_dist_ = min_expand_dist;
        max_expand_dist_ = max_expand_dist;
        min_dist_to_obs_ = min_dist_to_obs;
        points_min_dist_ = points_min_dist;
    }

    void sampling_2d::initialize(const nav_msgs::OccupancyGrid &map_data, const vector<float> &min_dist_to_obs_table)
    {
        map_data_ = map_data;
        min_dist_to_obs_table_ = min_dist_to_obs_table;

        int width = map_data_.info.width;
        int height = map_data_.info.height;
        float resolution = map_data_.info.resolution;

        current_map_x_ = width * resolution;
        current_map_y_ = height * resolution;

        float origin_x = map_data_.info.origin.position.x;
        float origin_y = map_data_.info.origin.position.y;
        start_x_ = origin_x + current_map_x_ / 2.0;
        start_y_ = origin_y + current_map_y_ / 2.0;

        origin_.x = 0.0f;
        origin_.y = 0.0f;
        origin_.z = 0.0f;

        rrt_tree_.indices.push_back(0);
        rrt_tree_.points.push_back(origin_);
        rrt_tree_.prev_indices.push_back(0);
        rrt_tree_.prev_points.push_back(origin_);
        rrt_tree_.next_indices.emplace_back();
        rrt_tree_.next_points.emplace_back();

        node_index_ = 0;

#if USE_R_TREE
        rtree_points_.clear();
        rtree_points_.emplace_back(origin_.x, origin_.y);
        vector<RTreeValue> initial_data;
        initial_data.emplace_back(rtree_points_.front(), 0);
        rtree_ = bgi::rtree<RTreeValue, bgi::quadratic<16>>(initial_data.begin(), initial_data.end());
#endif

        ROS_INFO("Reachability sampling initialized successfully.");
    }

    vector<geometry_msgs::Point32> sampling_2d::get_points(const int &timeout_secs)
    {
        ROS_INFO_STREAM("Starting " << (USE_RRT_SAMPLING ? "RRT sampling" : "random sampling") << ".");

        ros::Time start_time = ros::Time::now();

        vector<geometry_msgs::Point32> current_downsampled_points;
        vector<geometry_msgs::Point32> last_downsampled_points;
        size_t point_num_per_batch = static_cast<int>(((float)(map_data_.info.width * map_data_.info.height * pow(map_data_.info.resolution, 2)) /
                                                       (float)pow(points_min_dist_, 2)));
        point_num_per_batch = point_num_per_batch < 2000 ? 2000 : point_num_per_batch;

#if USE_RRT_SAMPLING
        while ((ros::Time::now() - start_time).toSec() <= timeout_secs)
        {
            for (int i = 0; i < point_num_per_batch; i++)
            {
                geometry_msgs::Point32 sample_point;
                sample_point.x = (generate_random_number() * current_map_x_) - (current_map_x_ / 2.0) + start_x_;
                sample_point.y = (generate_random_number() * current_map_y_) - (current_map_y_ / 2.0) + start_y_;
                sample_point.z = 0;

                tuple<int, geometry_msgs::Point32> nearest_point;

#if USE_R_TREE
                BGPoint query_pt(sample_point.x, sample_point.y);
                vector<RTreeValue> result;
                rtree_.query(bgi::nearest(query_pt, 1), back_inserter(result));
                get<0>(nearest_point) = result.front().second;
                get<1>(nearest_point) = rrt_tree_.points[get<0>(nearest_point)];
#else
                nearest_point = get_nearest_point(sample_point, rrt_tree_.points, rrt_tree_.indices);
#endif

                if (calculate_distance(get<1>(nearest_point), origin_) < 2 * min_dist_to_obs_ ||
                    !is_point_unknow(get<1>(nearest_point), map_data_, -1, -1))
                {
                    float expand_dist = calc_decayed_distance(get<1>(nearest_point),
                                                              map_data_, min_dist_to_obs_table_, min_dist_to_obs_,
                                                              min_expand_dist_, max_expand_dist_);

                    geometry_msgs::Point32 new_point = generate_point(get<1>(nearest_point), sample_point, expand_dist);

                    if (is_point_in_map(new_point, map_data_))
                    {
                        auto check_result = exploration_check_segment_type(get<1>(nearest_point), new_point, 3,
                                                                           min_dist_to_obs_, map_data_, min_dist_to_obs_table_, true,
                                                                           free_threshold_, occupied_threshold_);

                        if (check_result == exploration_segment_type::NO_COLLISION)
                        {
                            rrt_tree_.indices.push_back(++node_index_);
                            rrt_tree_.points.push_back(new_point);
                            rrt_tree_.prev_indices.push_back(get<0>(nearest_point));
                            rrt_tree_.prev_points.push_back(get<1>(nearest_point));
                            rrt_tree_.next_indices.emplace_back();
                            rrt_tree_.next_points.emplace_back();
                            rrt_tree_.next_indices[get<0>(nearest_point)].push_back(node_index_);
                            rrt_tree_.next_points[get<0>(nearest_point)].push_back(new_point);

#if USE_R_TREE
                            rtree_points_.emplace_back(new_point.x, new_point.y);
                            rtree_.insert(make_pair(rtree_points_.back(), node_index_));
#endif
                        }
                    }
                }
            }

            current_downsampled_points = resample_points(rrt_tree_.points, points_min_dist_);

            if (current_downsampled_points.size() - last_downsampled_points.size() <= 2)
            {
                break;
            }

            last_downsampled_points = current_downsampled_points;
        }

        return resample_points(rrt_tree_.points, points_min_dist_);
#else
        vector<geometry_msgs::Point32> points;

        while ((ros::Time::now() - start_time).toSec() <= timeout_secs)
        {
            for (int i = 0; i < point_num_per_batch; i++)
            {
                geometry_msgs::Point32 sample_point;
                sample_point.x = (generate_random_number() * current_map_x_) - (current_map_x_ / 2.0) + start_x_;
                sample_point.y = (generate_random_number() * current_map_y_) - (current_map_y_ / 2.0) + start_y_;
                sample_point.z = 0;

                if (is_point_in_map(sample_point, map_data_) &&
                    map_data_.data[position_to_grid_index(sample_point, map_data_)] >= 0 &&
                    map_data_.data[position_to_grid_index(sample_point, map_data_)] <= free_threshold_ &&
                    min_dist_to_obs_table_[position_to_grid_index(sample_point, map_data_)] >= min_dist_to_obs_)
                {
                    points.push_back(sample_point);
                }
            }

            current_downsampled_points = resample_points(points, points_min_dist_);

            if (current_downsampled_points.size() - last_downsampled_points.size() <= 2)
            {
                break;
            }

            last_downsampled_points = current_downsampled_points;
        }

        return resample_points(points, points_min_dist_);
#endif
    }

    float sampling_2d::generate_random_number()
    {
        return (*gen_random_number_)(*random_generator_);
    }
}