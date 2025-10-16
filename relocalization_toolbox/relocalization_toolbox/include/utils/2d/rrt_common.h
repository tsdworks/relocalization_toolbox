/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#ifndef RRT_COMMON_H
#define RRT_COMMON_H

#include <ros/ros.h>
#include <unordered_map>
#include <utils/math_utils.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>

using namespace std;

struct rrt_nodes
{
    vector<int> indices;
    vector<geometry_msgs::Point32> points;
    vector<int> prev_indices;
    vector<geometry_msgs::Point32> prev_points;
    vector<vector<int>> next_indices;
    vector<vector<geometry_msgs::Point32>> next_points;
};

#endif