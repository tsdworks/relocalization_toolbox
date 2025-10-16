/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <utils/2d/map_io_2d.h>

tuple<bool, nav_msgs::OccupancyGrid> load_map_2d(const string &path, const string &name, const string &map_frame)
{
    tuple<bool, nav_msgs::OccupancyGrid> ret;

    string yaml_file = path + "/" + name + ".yaml";
    string pgm_file = path + "/" + name + ".pgm";
    ifstream fin(yaml_file);

    if (!fin.is_open())
    {
        ROS_ERROR("2D map definition file is missing or inaccessible.");

        get<0>(ret) = false;

        return ret;
    }

    YAML::Node config = YAML::Load(fin);
    double resolution = config["resolution"].as<double>();
    vector<double> origin_vec = config["origin"].as<vector<double>>();
    int negate = config["negate"].as<int>();
    double occupied_thresh = config["occupied_thresh"].as<double>();
    double free_thresh = config["free_thresh"].as<double>();

    cv::Mat image = cv::imread(pgm_file, cv::IMREAD_UNCHANGED);

    if (image.empty())
    {
        ROS_ERROR("2D map image file is missing or inaccessible.");

        get<0>(ret) = false;

        return ret;
    }

    nav_msgs::OccupancyGrid map;
    map.info.resolution = resolution;
    map.info.width = image.cols;
    map.info.height = image.rows;
    map.info.origin.position.x = origin_vec[0];
    map.info.origin.position.y = origin_vec[1];
    map.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, origin_vec[2]);
    map.info.origin.orientation = tf2::toMsg(q);

    map.data.resize(map.info.width * map.info.height);

    for (int y = 0; y < image.rows; y++)
    {
        for (int x = 0; x < image.cols; x++)
        {
            uint8_t pixel = image.at<uint8_t>(image.rows - 1 - y, x);
            float occ = negate ? (255 - pixel) / 255.0 : pixel / 255.0;
            int index = y * image.cols + x;

            if (occ > occupied_thresh)
            {
                map.data[index] = 100;
            }
            else if (occ < free_thresh)
            {
                map.data[index] = 0;
            }
            else
            {
                map.data[index] = -1;
            }
        }
    }

    map.header.frame_id = map_frame;
    map.header.stamp = ros::Time::now();

    get<0>(ret) = true;
    get<1>(ret) = map;

    return ret;
}