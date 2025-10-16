/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <termios.h>
#include <algorithm>
#include <limits>
#include <angles/angles.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <common/common_state_code.h>
#include <utils/2d/relocalization_2d.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <relocalization_toolbox_msgs/initial_pose_request.h>
#include <relocalization_toolbox_msgs/relocalization_request.h>

#define NODE_NAME "relocalization_2d_handler"
#define TAG "2D Global Relocalization"

using namespace std;

// general parameters
bool publish_init_pose_topic;
bool call_init_pose_request;

string map_frame;
string base_frame;
string lidar_frame;
bool prefetch_map_from_topic;
bool lidar_reverted;
int lidar_sampling_step;

bool enable_visualization;

bool use_reachability_sampling;
float rrt_min_expand_dist, rrt_max_expand_dist;
int reachability_sampling_duration;

float fit_score_threshold_min, fit_score_threshold_max;
float angle_search_step;
int area_batch_proc_number;
int area_candidate_number;
float min_dist_to_obstacle;

// likelihood field model parameters
float sigma_hit;
float z_hit;
float z_rand;
float max_tolerance_dist;

// map-related parameters
int map_free_threshold;
int map_occupied_threshold;

// submap partitioning parameters
float sensing_radius;
float anchor_point_min_dist;

// ICP matching parameters
float max_correspondence_distance;
float transformation_epsilon;
float euclidean_fitness_epsilon;
int max_iterations;

// tf components and transforms
unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
unique_ptr<tf2_ros::Buffer> tf_buffer;
geometry_msgs::TransformStamped tf_lidar_to_base;

// relocalization services
ros::ServiceServer relocalization_server;
ros::ServiceServer relocalization_simple_server;
// relocalization client and publisher
ros::ServiceClient set_init_pose_client;
ros::Publisher init_pose_pub;

// relocalization core system
unique_ptr<relocalization_2d::relocalization_2d> relocalization_2d_instance;

// topic data
sensor_msgs::LaserScan scan_data;
nav_msgs::OccupancyGrid map_data;
bool map_data_received = false;

// callback functions
bool relocalization_request_callback(relocalization_toolbox_msgs::relocalization_request::Request &req,
                                     relocalization_toolbox_msgs::relocalization_request::Response &res)
{
    bool ret = false;

    // get current scan
    auto scan_data_ptr = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", ros::Duration(2.0));

    if (scan_data_ptr)
    {
        scan_data = *scan_data_ptr;
        scan_data.header.frame_id = lidar_frame;

        // check request mode
        if (req.mode == req.MODE_MAP_FROM_FILE)
        {
            // load map from file
            auto data_loaded = load_map_2d(req.path, req.name, map_frame);
            map_data_received = get<0>(data_loaded);

            if (map_data_received)
            {
                map_data = get<1>(data_loaded);
            }
        }
        else if (req.mode == req.MODE_MAP_FROM_TOPIC)
        {
            // load map from topic
            auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(5.0));

            if (map_data_ptr)
            {
                map_data = *map_data_ptr;
                map_data.header.frame_id = map_frame;
                map_data_received = true;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed, invalid map input mode.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }

        if (map_data_received)
        {
            // Start to relocalize
            auto reloc_result = relocalization_2d_instance->relocalize(scan_data, lidar_reverted, lidar_sampling_step, map_data);

            if (get<0>(reloc_result))
            {
                res.result = CMD_RESPONSE_OK;
                res.score = get<1>(reloc_result);

                // get transformations
                tf2::Transform tf_map_lidar, tf_lidar_base, tf_map_base;
                tf2::fromMsg(get<2>(reloc_result).transform, tf_map_lidar);
                tf2::fromMsg(tf_lidar_to_base.transform, tf_lidar_base);
                tf_map_base = tf_map_lidar * tf_lidar_base;
                geometry_msgs::TransformStamped tf_map_to_base;
                tf_map_to_base.header.stamp = get<2>(reloc_result).header.stamp;
                tf_map_to_base.header.frame_id = map_frame;
                tf_map_to_base.child_frame_id = base_frame;
                tf_map_to_base.transform = tf2::toMsg(tf_map_base);

                res.tf_map_to_base = tf_map_to_base;

                geometry_msgs::PoseWithCovarianceStamped init_pose;
                init_pose.header.stamp = ros::Time::now();
                init_pose.header.frame_id = map_frame;
                init_pose.pose.pose.position.x = res.tf_map_to_base.transform.translation.x;
                init_pose.pose.pose.position.y = res.tf_map_to_base.transform.translation.y;
                init_pose.pose.pose.position.z = res.tf_map_to_base.transform.translation.z;
                init_pose.pose.pose.orientation = res.tf_map_to_base.transform.rotation;

                if (publish_init_pose_topic)
                {
                    init_pose_pub.publish(init_pose);
                }

                if (call_init_pose_request)
                {
                    relocalization_toolbox_msgs::initial_pose_request init_pose_request;
                    init_pose_request.request.pose = init_pose;

                    set_init_pose_client.call(init_pose_request);
                }
            }
            else
            {
                ROS_ERROR("%s: Relocalization failed. Score below threshold.", TAG);

                res.result = CMD_RESPONSE_ERROR;
            }
        }
        else
        {
            ROS_ERROR("%s: Request failed. No map data available.", TAG);

            res.result = CMD_RESPONSE_ERROR;
        }
    }
    else
    {
        ROS_ERROR("%s: Request failed. No scan data received.", TAG);

        res.result = CMD_RESPONSE_ERROR;
    }

    ret = true;

    return ret;
}

bool relocalization_simple_request_callback(std_srvs::Empty::Request &req,
                                            std_srvs::Empty::Response &res)
{
    relocalization_toolbox_msgs::relocalization_request srv;
    srv.request.mode = srv.request.MODE_MAP_FROM_TOPIC;

    return relocalization_request_callback(srv.request, srv.response);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);

    setlocale(LC_CTYPE, "en_US.utf8");

    ROS_INFO("%s: Node started.", TAG);

    ros::NodeHandle node_handle;
    ros::NodeHandle node_handle_param("~/relocalization_2d");

    // Load parameters
    node_handle_param.param<bool>("publish_init_pose_topic", publish_init_pose_topic, false);
    node_handle_param.param<bool>("call_init_pose_request", call_init_pose_request, true);

    node_handle_param.param<string>("map_frame", map_frame, "map");
    node_handle_param.param<string>("base_frame", base_frame, "base_link");
    node_handle_param.param<string>("lidar_frame", lidar_frame, "lidar_link");

    node_handle_param.param<bool>("prefetch_map_from_topic", prefetch_map_from_topic, true);

    node_handle_param.param<bool>("lidar_reverted", lidar_reverted, true);

    node_handle_param.param<int>("lidar_sampling_step", lidar_sampling_step, 4);

    node_handle_param.param<bool>("enable_visualization", enable_visualization, true);

    node_handle_param.param<bool>("use_reachability_sampling", use_reachability_sampling, false);
    node_handle_param.param<float>("rrt_min_expand_dist", rrt_min_expand_dist, 0.5);
    node_handle_param.param<float>("rrt_max_expand_dist", rrt_max_expand_dist, 1.0);
    node_handle_param.param<int>("reachability_sampling_duration", reachability_sampling_duration, 5);

    node_handle_param.param<float>("fit_score_threshold_max", fit_score_threshold_max, 0.9f);
    node_handle_param.param<float>("fit_score_threshold_min", fit_score_threshold_min, 0.8f);
    node_handle_param.param<float>("angle_search_step", angle_search_step, 10.0f * M_PI / 180.0f);
    node_handle_param.param<int>("area_batch_proc_number", area_batch_proc_number, 500);
    node_handle_param.param<int>("area_candidate_number", area_candidate_number, 10);
    node_handle_param.param<float>("min_dist_to_obstacle", min_dist_to_obstacle, 0.4);

    node_handle_param.param<float>("sigma_hit", sigma_hit, 0.2f);
    node_handle_param.param<float>("z_hit", z_hit, 1.0f);
    node_handle_param.param<float>("z_rand", z_rand, 0.0f);
    node_handle_param.param<float>("max_tolerance_dist", max_tolerance_dist, 1.0f);

    node_handle_param.param<int>("map_free_threshold", map_free_threshold, 30);
    node_handle_param.param<int>("map_occupied_threshold", map_occupied_threshold, 55);

    node_handle_param.param<float>("sensing_radius", sensing_radius, 15.0f);
    node_handle_param.param<float>("anchor_point_min_dist", anchor_point_min_dist, 3.0f);

    node_handle_param.param<float>("max_correspondence_distance", max_correspondence_distance, 1.0f);
    node_handle_param.param<float>("transformation_epsilon", transformation_epsilon, 1e-6);
    node_handle_param.param<float>("euclidean_fitness_epsilon", euclidean_fitness_epsilon, 1e-5);
    node_handle_param.param<int>("max_iterations", max_iterations, 50);

    // tf setup
    tf_broadcaster = unique_ptr<tf2_ros::TransformBroadcaster>(new tf2_ros::TransformBroadcaster);
    tf_buffer = unique_ptr<tf2_ros::Buffer>(new tf2_ros::Buffer);
    tf2_ros::TransformListener tf_listener(*tf_buffer);

    // initialize relocalization instance
    relocalization_2d_instance = unique_ptr<relocalization_2d::relocalization_2d>(
        new relocalization_2d::relocalization_2d(
            enable_visualization, use_reachability_sampling,
            rrt_min_expand_dist, rrt_max_expand_dist, reachability_sampling_duration,
            fit_score_threshold_min, fit_score_threshold_max, angle_search_step,
            area_batch_proc_number, area_candidate_number, min_dist_to_obstacle,
            sigma_hit, z_hit, z_rand, max_tolerance_dist,
            map_free_threshold, map_occupied_threshold,
            sensing_radius, anchor_point_min_dist,
            max_correspondence_distance, transformation_epsilon, euclidean_fitness_epsilon, max_iterations));

    if (prefetch_map_from_topic)
    {
        ROS_INFO("%s: Attempting to fetch map data...", TAG);

        auto map_data_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(30.0));

        if (map_data_ptr)
        {
            ROS_INFO("%s: Map data received successfully.", TAG);

            map_data = *map_data_ptr;
            map_data.header.frame_id = map_frame;
            map_data_received = true;

            relocalization_2d_instance->set_map(map_data);
        }
        else
        {
            ROS_WARN("%s: No map data. Will fetch when service is called.", TAG);
        }
    }

    // initialize services and publishers
    relocalization_server = node_handle.advertiseService("relocalization_request", relocalization_request_callback);
    relocalization_simple_server = node_handle.advertiseService("relocalization_simple_request", relocalization_simple_request_callback);
    set_init_pose_client = node_handle.serviceClient<relocalization_toolbox_msgs::initial_pose_request>("set_init_pose_request");
    init_pose_pub = node_handle.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    ROS_INFO("%s: Waiting for static transform from %s to %s...", TAG, lidar_frame.c_str(), base_frame.c_str());

    try
    {
        tf_lidar_to_base = tf_buffer->lookupTransform(
            lidar_frame,
            base_frame,
            ros::Time(0),
            ros::Duration(10.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_ERROR("%s: Failed to retrieve transform from %s to %s.", TAG,
                  lidar_frame.c_str(), base_frame.c_str());

        return 0;
    }

    ROS_INFO("%s: Ready and listening for service requests.", TAG);

    ros::spin();

    if (relocalization_2d_instance)
    {
        relocalization_2d_instance.reset();
    }

    return 0;
}