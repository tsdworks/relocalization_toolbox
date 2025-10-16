/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <utils/2d/relocalization_2d.h>

namespace relocalization_2d
{
    relocalization_2d::relocalization_2d(
        const bool &enable_visualization, const bool &use_reachability_sampling,
        const float &rrt_min_expand_dist, const float &rrt_max_expand_dist, const int &reachability_sampling_duration,
        const float &fit_score_threshold_min, const float &fit_score_threshold_max,
        const float &angle_search_step, const int &area_batch_proc_number, const int &area_candidate_number,
        const float &min_dist_to_obstacle,
        const float &sigma_hit, const float &z_hit, const float &z_rand, const float &max_tolerance_dist,
        const int &map_free_threshold, const int &map_occupied_threshold,
        const float &sensing_radius, const float anchor_point_min_dist,
        const float &max_correspondence_distance, const float &transformation_epsilon, const float &euclidean_fitness_epsilon, const int &max_iterations)
        : enable_visualization_(enable_visualization), use_reachability_sampling_(use_reachability_sampling),
          rrt_min_expand_dist_(rrt_min_expand_dist), rrt_max_expand_dist_(rrt_max_expand_dist), reachability_sampling_duration_(reachability_sampling_duration),
          fit_score_threshold_min_(fit_score_threshold_min), fit_score_threshold_max_(fit_score_threshold_max),
          angle_search_step_(angle_search_step), area_batch_proc_number_(area_batch_proc_number), area_candidate_number_(area_candidate_number),
          min_dist_to_obstacle_(min_dist_to_obstacle),
          sigma_hit_(sigma_hit), z_hit_(z_hit), z_rand_(z_rand), max_tolerance_dist_(max_tolerance_dist),
          map_free_threshold_(map_free_threshold), map_occupied_threshold_(map_occupied_threshold),
          sensing_radius_(sensing_radius), anchor_point_min_dist_(anchor_point_min_dist),
          max_correspondence_distance_(max_correspondence_distance), transformation_epsilon_(transformation_epsilon), euclidean_fitness_epsilon_(euclidean_fitness_epsilon), max_iterations_(max_iterations)
    {
        sensing_radius_ += anchor_point_min_dist_;

        submap_set_2d_instance_ = unique_ptr<submap_set_2d::submap_set_2d>(new submap_set_2d::submap_set_2d(
            use_reachability_sampling,
            rrt_min_expand_dist, rrt_max_expand_dist, reachability_sampling_duration,
            map_free_threshold_, map_occupied_threshold_,
            sensing_radius_, anchor_point_min_dist_, angle_search_step_));

        likelihood_field_2d_instance_ = unique_ptr<likelihood_field_2d::likelihood_field_2d>(new likelihood_field_2d::likelihood_field_2d(
            sigma_hit_, z_hit_, z_rand_, map_occupied_threshold_));

        gicp_2d_instance_ = unique_ptr<gicp_2d::gicp_2d>(new gicp_2d::gicp_2d(
            max_correspondence_distance_,
            transformation_epsilon_,
            euclidean_fitness_epsilon_,
            max_iterations_,
            map_occupied_threshold_));

        submap_pub_ = node_handle_.advertise<nav_msgs::OccupancyGrid>(
            "submaps", 1);

        return;
    };

    float relocalization_2d::calc_confidence(
        const nav_msgs::OccupancyGrid &map,
        const float &likelihood_field_score, const float &mean_min_dist, const float &consistency_score)
    {
        float p = exp(likelihood_field_score);

        float resolution = map.info.resolution;
        float min_dist = resolution * 0.5f;
        float max_dist = max_tolerance_dist_;

        float clamped_dist = min(max(mean_min_dist, min_dist), max_dist);
        float d_score = 1.0f - ((clamped_dist - min_dist) / (max_dist - min_dist));

        float confidence = pow(p * d_score * consistency_score, 1.0f / 3.0f);

        return confidence;
    }

    void relocalization_2d::set_map(const nav_msgs::OccupancyGrid &map)
    {
        ROS_INFO("Updating relocalization map data...");

        map_data_ = map;

        obs_dist_table_global_ = calc_min_dist_to_obs_table(
            map_data_, map_occupied_threshold_);

        gicp_2d_instance_->set_map(map_data_);

        ROS_INFO("Relocalization map data loaded successfully.");
    }

    tuple<bool, float, geometry_msgs::TransformStamped> relocalization_2d::relocalize(
        const sensor_msgs::LaserScan &scan,
        const bool &lidar_reverted,
        const int &lidar_sampling_step,
        const nav_msgs::OccupancyGrid &map)
    {
        tuple<bool, float, geometry_msgs::TransformStamped> ret;
        get<0>(ret) = false;
        get<1>(ret) = 0.0f;

        // calculate distance field of the map
        bool map_changed = false;

        if (map_data_.header.frame_id != map.header.frame_id ||
            map_data_.info.height != map.info.height ||
            map_data_.info.width != map.info.width ||
            map_data_.data != map.data)
        {
            set_map(map);

            map_changed = true;
        }
        else
        {
            map_data_ = map;
        }

        if (!map_changed)
        {
            ROS_INFO("Using cached relocalization map data.");
        }
        else
        {
            ROS_INFO("Relocalization map data updated.");
        }

        // start sample
        auto sampling_start_time = ros::WallTime::now();

        bool sampling_changed = false;

        if (map_changed ||
            scan_data_.header.frame_id != scan.header.frame_id ||
            abs(scan_data_.angle_min - scan.angle_min) > 1e-3 ||
            abs(scan_data_.angle_max - scan.angle_max) > 1e-3 ||
            scan_data_.ranges.size() != scan.ranges.size() ||
            lidar_reverted_ != lidar_reverted ||
            lidar_sampling_step_ != lidar_sampling_step)
        {
            ROS_INFO("Updating relocalization sampling data...");

            scan_data_ = scan;
            lidar_reverted_ = lidar_reverted;
            lidar_sampling_step_ = lidar_sampling_step;

            submap_set_raw_ = submap_set_2d_instance_->get_submap_set(map_data_, scan_data_, lidar_reverted_, lidar_sampling_step_,
                                                                      obs_dist_table_global_, min_dist_to_obstacle_,
                                                                      enable_visualization_);

            sampling_changed = true;
        }
        else
        {
            scan_data_ = scan;
            lidar_reverted_ = lidar_reverted;
            lidar_sampling_step_ = lidar_sampling_step;
        }

        if (!sampling_changed)
        {
            ROS_INFO("Using cached relocalization sampling data.");
        }
        else
        {
            ROS_INFO("Relocalization sampling data updated.");
        }

        gicp_2d_instance_->set_scan(scan_data_);

        float scan_mean_dist = calc_laser_mean_dist(scan_data_, sensing_radius_, lidar_sampling_step_);

        auto sampling_end_time = ros::WallTime::now();

        ROS_INFO("Total %ld observation regions available.", submap_set_raw_.size());

        if (submap_set_raw_.size() > 0)
        {
            ROS_INFO("Starting optimal region matching...");

            submap_set_.clear();

            vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, float>> submap_set_parallel;

#pragma omp parallel
            {
                vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, float>> local_result;

#pragma omp for nowait
                for (int i = 0; i < static_cast<int>(submap_set_raw_.size()); i++)
                {
                    const auto &submap_raw = submap_set_raw_[i];
                    float min_diff = numeric_limits<float>::max();
                    float best_value = 0.0f;

                    for (const auto &val : get<2>(submap_raw))
                    {
                        float diff = abs(val - scan_mean_dist);

                        if (diff < min_diff)
                        {
                            min_diff = diff;
                            best_value = val;
                        }
                    }

                    local_result.emplace_back(get<0>(submap_raw), get<1>(submap_raw), best_value);
                }

#pragma omp critical
                submap_set_parallel.insert(submap_set_parallel.end(),
                                           local_result.begin(), local_result.end());
            }

            submap_set_ = move(submap_set_parallel);

            // sort by scan feature
            sort(submap_set_.begin(), submap_set_.end(),
                 [scan_mean_dist](const auto &a, const auto &b)
                 {
                     return abs(get<2>(a) - scan_mean_dist) < abs(get<2>(b) - scan_mean_dist);
                 });

            size_t submap_set_size = submap_set_.size();
            size_t start_idx = 0;
            int batch_idx = 0;
            geometry_msgs::TransformStamped trans_best;
            float loc_conf_best = 0;
            float mean_dist_best = 0;
            bool found_valid = false;

            while (start_idx < submap_set_size)
            {
                ROS_INFO("Starting batch round %d...", ++batch_idx);

                size_t end_idx = min(start_idx + area_batch_proc_number_, submap_set_size);

                vector<tuple<geometry_msgs::Point32, nav_msgs::OccupancyGrid, float>> submap_batch(
                    submap_set_.begin() + start_idx, submap_set_.begin() + end_idx);

                size_t submap_batch_size = submap_batch.size();

                vector<tuple<float, geometry_msgs::TransformStamped, nav_msgs::OccupancyGrid, float>> candidate_submaps;

#pragma omp parallel
                {
                    vector<tuple<float, geometry_msgs::TransformStamped, nav_msgs::OccupancyGrid, float>> local_submaps;

#pragma omp for nowait
                    for (int i = 0; i < submap_batch_size; i++)
                    {
                        geometry_msgs::TransformStamped tf_map_to_lidar;
                        tf_map_to_lidar.header.frame_id = map_data_.header.frame_id;
                        tf_map_to_lidar.header.stamp = scan_data_.header.stamp;
                        tf_map_to_lidar.child_frame_id = scan_data_.header.frame_id;
                        tf_map_to_lidar.transform.translation.x = get<0>(submap_batch[i]).x;
                        tf_map_to_lidar.transform.translation.y = get<0>(submap_batch[i]).y;
                        tf_map_to_lidar.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(lidar_reverted_ ? M_PI : 0, 0, 0);

                        geometry_msgs::TransformStamped tf_map_to_lidar_area_best = tf_map_to_lidar;
                        float confidence_score_area_best = -INFINITY;

                        int yaw_steps = static_cast<int>(2 * M_PI / angle_search_step_);

                        for (int j = 0; j < yaw_steps; j++)
                        {
                            float yaw = j * angle_search_step_;

                            geometry_msgs::TransformStamped tf_map_to_lidar_search = tf_map_to_lidar;
                            tf_map_to_lidar_search.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(lidar_reverted_ ? M_PI : 0, 0, yaw);

                            auto matching_result = likelihood_field_2d_instance_->get_likelihood_field_score(
                                tf_map_to_lidar_search,
                                scan_data_, map_data_, obs_dist_table_global_, lidar_sampling_step,
                                enable_visualization_);

                            float confidence_score = calc_confidence(map_data_,
                                                                     get<0>(matching_result), get<1>(matching_result), get<2>(matching_result));

                            if (confidence_score > confidence_score_area_best)
                            {
                                tf_map_to_lidar_area_best = tf_map_to_lidar_search;
                                confidence_score_area_best = confidence_score;
                            }
                        }

                        tuple<float, geometry_msgs::TransformStamped, nav_msgs::OccupancyGrid, float> candidate_submap;
                        get<0>(candidate_submap) = confidence_score_area_best;
                        get<1>(candidate_submap) = tf_map_to_lidar_area_best;
                        get<2>(candidate_submap) = get<1>(submap_batch[i]);
                        get<3>(candidate_submap) = get<2>(submap_batch[i]);

                        local_submaps.emplace_back(candidate_submap);
                    }

#pragma omp critical
                    candidate_submaps.insert(candidate_submaps.end(), local_submaps.begin(), local_submaps.end());
                }

                // sort region set
                ROS_INFO("Batch %d: Sorting region candidates...", batch_idx);

                vector<tuple<float, geometry_msgs::TransformStamped, nav_msgs::OccupancyGrid, float>> candidate_transformations;

                size_t candidate_submap_size = candidate_submaps.size();

                sort(candidate_submaps.begin(), candidate_submaps.end(),
                     [](const auto &a, const auto &b)
                     {
                         return get<0>(a) > get<0>(b);
                     });

#pragma omp parallel for schedule(dynamic)
                for (int i = 0; i < min(candidate_submap_size, (size_t)area_candidate_number_); i++)
                {
                    ROS_INFO("Batch %d: Trying region %d, anchor point (%.3f, %.3f, %.3f), score: %.4f.",
                             batch_idx, i,
                             get<1>(candidate_submaps[i]).transform.translation.x,
                             get<1>(candidate_submaps[i]).transform.translation.y,
                             tf::getYaw(get<1>(candidate_submaps[i]).transform.rotation),
                             get<0>(candidate_submaps[i]));

                    ROS_INFO("Batch %d: Starting ICP alignment...", batch_idx);

                    geometry_msgs::TransformStamped tf_map_to_lidar_aligned = gicp_2d_instance_->match(get<1>(candidate_submaps[i]));

                    ROS_INFO("Batch %d: Recomputing confidence score...", batch_idx);

                    // re-calculate confidence score
                    auto matching_result = likelihood_field_2d_instance_->get_likelihood_field_score(
                        tf_map_to_lidar_aligned,
                        scan_data_, map_data_, obs_dist_table_global_, lidar_sampling_step,
                        enable_visualization_);

                    float confidence_score = calc_confidence(map_data_,
                                                             get<0>(matching_result), get<1>(matching_result), get<2>(matching_result));

                    tuple<float, geometry_msgs::TransformStamped, nav_msgs::OccupancyGrid, float> candidate_transformation;
                    get<0>(candidate_transformation) = confidence_score;
                    get<1>(candidate_transformation) = tf_map_to_lidar_aligned;
                    get<2>(candidate_transformation) = get<2>(candidate_submaps[i]);
                    get<3>(candidate_transformation) = get<3>(candidate_submaps[i]);

                    ROS_INFO("Batch %d: Registration completed, pose (%.3f, %.3f, %.3f), score: %.4f.",
                             batch_idx,
                             tf_map_to_lidar_aligned.transform.translation.x,
                             tf_map_to_lidar_aligned.transform.translation.y,
                             tf::getYaw(tf_map_to_lidar_aligned.transform.rotation),
                             confidence_score);

#pragma omp critical
                    {
                        candidate_transformations.emplace_back(candidate_transformation);
                    }
                }

                ROS_INFO("Batch %d: Sorting result candidates...", batch_idx);

                sort(candidate_transformations.begin(), candidate_transformations.end(),
                     [](const auto &a, const auto &b)
                     {
                         return get<0>(a) > get<0>(b);
                     });

                ROS_INFO("Batch %d: Best estimate in this batch: (%.3f, %.3f, %.3f), score: %.4f.",
                         batch_idx,
                         get<1>(candidate_transformations[0]).transform.translation.x,
                         get<1>(candidate_transformations[0]).transform.translation.y,
                         tf::getYaw(get<1>(candidate_transformations[0]).transform.rotation),
                         get<0>(candidate_transformations[0]));

                if (get<0>(candidate_transformations[0]) >= fit_score_threshold_max_)
                {
                    ROS_INFO("Batch %d: Confidence reached threshold, ending this batch.", batch_idx);

                    loc_conf_best = get<0>(candidate_transformations[0]);
                    trans_best = get<1>(candidate_transformations[0]);
                    mean_dist_best = get<3>(candidate_transformations[0]);

                    found_valid = true;

                    break;
                }
                else if (get<0>(candidate_transformations[0]) >= fit_score_threshold_min_)
                {
                    if (get<0>(candidate_transformations[0]) >= loc_conf_best)
                    {
                        ROS_INFO("Batch %d: Result score %.4f exceeds previous best %.4f, updating final result.",
                                 batch_idx,
                                 get<0>(candidate_transformations[0]), loc_conf_best);

                        loc_conf_best = get<0>(candidate_transformations[0]);
                        trans_best = get<1>(candidate_transformations[0]);
                        mean_dist_best = get<3>(candidate_transformations[0]);
                    }
                    else
                    {
                        ROS_INFO("Batch %d: Result score %.4f will not be used to update final result.",
                                 batch_idx,
                                 get<0>(candidate_transformations[0]));
                    }

                    found_valid = true;
                }
                else
                {
                    ROS_WARN("Batch %d: Result score %.4f is below threshold %.4f, batch discarded.",
                             batch_idx,
                             get<0>(candidate_transformations[0]), fit_score_threshold_min_);

                    if (get<0>(candidate_transformations[0]) >= loc_conf_best)
                    {
                        loc_conf_best = get<0>(candidate_transformations[0]);
                        trans_best = get<1>(candidate_transformations[0]);
                        mean_dist_best = get<3>(candidate_transformations[0]);
                    }
                }

                start_idx = end_idx;
            }

            // update final result
            get<1>(ret) = loc_conf_best;
            get<2>(ret) = trans_best;

            if (found_valid)
            {
                ROS_INFO("\033[32mFinal best estimate: (%.3f, %.3f, %.3f), score: %.4f.\033[0m",
                         get<2>(ret).transform.translation.x,
                         get<2>(ret).transform.translation.y,
                         tf::getYaw(get<2>(ret).transform.rotation),
                         get<1>(ret));

                get<0>(ret) = true;
            }
            else
            {
                ROS_ERROR("All batch results below score threshold. Final score: %.4f, minimum required: %.4f. Relocalization failed.",
                          get<1>(ret), fit_score_threshold_min_);

                get<0>(ret) = false;
            }
        }
        else
        {
            ROS_ERROR("No region available.");
        }

        auto matching_end_time = ros::WallTime::now();

        ROS_INFO("Sampling time: %.3lfs, matching time: %.3lfs.",
                 (sampling_end_time - sampling_start_time).toSec(),
                 (matching_end_time - sampling_end_time).toSec());

        return ret;
    }
};