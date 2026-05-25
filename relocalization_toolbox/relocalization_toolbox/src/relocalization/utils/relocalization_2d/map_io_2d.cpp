/**
 * @author
 *   ZHANG Muhua
 * @affiliation
 *   Southwest Jiaotong University
 * @license
 *    GPL-3.0 License
 */

#include <relocalization/utils/relocalization_2d/map_io_2d.h>

tuple<bool, nav_msgs::OccupancyGrid> load_map_2d(const string &path, const string &name, const string &map_frame)
{
    tuple<bool, nav_msgs::OccupancyGrid> ret;
    get<0>(ret) = false;

    auto join_path = [](const string &dir, const string &file) -> string
    {
        if (file.empty())
        {
            return dir;
        }

        if (file[0] == '/')
        {
            return file;
        }

        if (dir.empty() || dir.back() == '/')
        {
            return dir + file;
        }

        return dir + "/" + file;
    };

    string yaml_file = join_path(path, name + ".yaml");

    YAML::Node config;
    try
    {
        config = YAML::LoadFile(yaml_file);
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("2D map definition file is missing, inaccessible, or invalid: "
                         << yaml_file << ", error: " << e.what());
        return ret;
    }

    if (!config["resolution"] || !config["origin"])
    {
        ROS_ERROR_STREAM("2D map yaml is missing required fields: " << yaml_file);
        return ret;
    }

    double resolution = 0.0;
    vector<double> origin_vec;

    try
    {
        resolution = config["resolution"].as<double>();
        origin_vec = config["origin"].as<vector<double>>();
    }
    catch (const std::exception &e)
    {
        ROS_ERROR_STREAM("2D map yaml field parse failed: "
                         << yaml_file << ", error: " << e.what());
        return ret;
    }

    if (resolution <= 0.0)
    {
        ROS_ERROR_STREAM("2D map resolution is invalid: " << resolution);
        return ret;
    }

    if (origin_vec.size() < 3)
    {
        ROS_ERROR_STREAM("2D map origin must contain [x, y, yaw].");
        return ret;
    }

    int negate = config["negate"] ? config["negate"].as<int>() : 0;
    double occupied_thresh = config["occupied_thresh"] ? config["occupied_thresh"].as<double>() : 0.65;
    double free_thresh = config["free_thresh"] ? config["free_thresh"].as<double>() : 0.196;

    string image_name = config["image"] ? config["image"].as<string>() : name + ".pgm";
    string pgm_file = join_path(path, image_name);

    cv::Mat image = cv::imread(pgm_file, cv::IMREAD_GRAYSCALE);

    if (image.empty())
    {
        ROS_ERROR_STREAM("2D map image file is missing or inaccessible: " << pgm_file);

        return ret;
    }

    if (image.type() != CV_8UC1)
    {
        ROS_ERROR_STREAM("2D map image must be 8-bit grayscale image. file: " << pgm_file);

        return ret;
    }

    ros::Time now = ros::Time::now();

    nav_msgs::OccupancyGrid map;
    map.header.frame_id = map_frame;
    map.header.stamp = now;

    map.info.map_load_time = now;
    map.info.resolution = resolution;
    map.info.width = image.cols;
    map.info.height = image.rows;

    map.info.origin.position.x = origin_vec[0];
    map.info.origin.position.y = origin_vec[1];
    map.info.origin.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, origin_vec[2]);
    map.info.origin.orientation = tf2::toMsg(q);

    map.data.resize(map.info.width * map.info.height, -1);

    for (int y = 0; y < image.rows; ++y)
    {
        const uint8_t *row_ptr = image.ptr<uint8_t>(image.rows - 1 - y);

        for (int x = 0; x < image.cols; ++x)
        {
            uint8_t pixel = row_ptr[x];

            float occ = negate ? static_cast<float>(pixel) / 255.0f
                               : static_cast<float>(255 - pixel) / 255.0f;

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

    get<0>(ret) = true;
    get<1>(ret) = map;

    return ret;
}