#include "uvs_mapserver.hpp"

#include "tf2/utils.h"

UVSMapServer::UVSMapServer()
    : Node("uvs_mapserver")
{
    declare_parameter<std::string>("world_name", "jxl3028");
    std::string world_name = get_parameter("world_name").as_string();
    loadWorld(world_name, world_);

    // 生成静态地图
    RCLCPP_INFO(get_logger(), "Generating static map...");
    // 1. 获得地面的形状
    // 1.1 确定一个外接矩形
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    for (auto &point : world_.ground.shape.points)
    {
        min_x = std::min(min_x, point.x + world_.ground.origin.x);
        min_y = std::min(min_y, point.y + world_.ground.origin.y);
        max_x = std::max(max_x, point.x + world_.ground.origin.x);
        max_y = std::max(max_y, point.y + world_.ground.origin.y);
    }
    // 1.2 矩形加边，确定最终的矩形边界
    min_x -= 2 * world_.ground.resolutionX;
    min_y -= 2 * world_.ground.resolutionY;
    max_x += 2 * world_.ground.resolutionX;
    max_y += 2 * world_.ground.resolutionY;

    int width = (max_x - min_x) / world_.ground.resolutionX;
    int height = (max_y - min_y) / world_.ground.resolutionY;
    // 1.3 申请地图空间，初始化全占用
    map_static_.header.frame_id = "map";
    map_static_.info.width = width;
    map_static_.info.height = height;
    map_static_.info.resolution = world_.ground.resolutionX;
    map_static_.info.origin.position.x = min_x;
    map_static_.info.origin.position.y = min_y;
    map_static_.info.origin.position.z = 0;
    map_static_.info.origin.orientation.x = 0;
    map_static_.info.origin.orientation.y = 0;
    map_static_.info.origin.orientation.z = 0;
    map_static_.info.origin.orientation.w = 1;
    map_static_.data.resize(width * height, 100);
    // 1.4 多边形填充，将多边形内部的点设置为空闲
    std::vector<Point2I> polygon_ground = getFootprint(world_.ground.shape, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(min_x, min_y));
    for (auto &point : polygon_ground)
    {
        map_static_.data[point.y * width + point.x] = 0;
    }
    // 2. 遍历所有障碍物，填充静态占用
    for (auto &obstacle : world_.obstacles)
    {
        std::vector<Point2I> polygon_obstacle = getFootprint(obstacle.footprint, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(min_x, min_y));
        for (auto &point : polygon_obstacle)
        {
            if (point.x < 0 || point.x >= width || point.y < 0 || point.y >= height)
            {
                continue;
            }
            map_static_.data[point.y * width + point.x] = 100;
        }
    }
    // 3. 生成动态物体的形状
    for (auto &cargo : world_.cargos)
    {
        std::vector<Point2I> polygon_cargo = getFootprint(cargo.shape, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(min_x, min_y));
        cargo_shape_[cargo.name] = std::make_pair(cargo.shape, std::vector<Point2I>());
    }

    uv_query_world_service_ = create_service<uvs_message::srv::UvQueryWorld>("uvs_query_world", 
        std::bind(&UVSMapServer::uvQueryWorldCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // uv_query_element_service_ = create_service<uvs_message::srv::UvQueryElement>("uv_query_element", 
    //     std::bind(&UVSMapServer::uvQueryElementCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    // uv_query_map_service_ = create_service<uvs_message::srv::UvQueryMap>("uv_query_map",
    //     std::bind(&UVSMapServer::uvQueryMapCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
    uv_opt_pose_list_sub_ = create_subscription<uvs_message::msg::UvOptPoseList>("uvs_pose_list",
        10, std::bind(&UVSMapServer::uvOptPoseListCallback, this, std::placeholders::_1));

    map_static_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map", 10);
    timer_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(std::chrono::milliseconds(33), std::bind(&UVSMapServer::timerCallback, this), timer_callback_group_);

    RCLCPP_INFO(get_logger(), "UVSMapServer is ready.");
}

UVSMapServer::~UVSMapServer()
{
}

void UVSMapServer::uvQueryWorldCallback(const std::shared_ptr<rmw_request_id_t> request_header, 
                                        const std::shared_ptr<uvs_message::srv::UvQueryWorld::Request> request, 
                                        std::shared_ptr<uvs_message::srv::UvQueryWorld::Response> response)
{
    response->world_name = world_.name;

    uvs_message::msg::DscpGround ground;
    ground.origin.x = world_.ground.origin.x;
    ground.origin.y = world_.ground.origin.y;
    ground.origin.z = 0;
    ground.resolution_x = world_.ground.resolutionX;
    ground.resolution_y = world_.ground.resolutionY;
    for (auto &point : world_.ground.shape.points)
    {
        geometry_msgs::msg::Point32 p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0;
        ground.shape.points.push_back(p);
    }
    response->ground = ground;

    for (auto &obstacle : world_.obstacles)
    {
        uvs_message::msg::DscpObstacle obs;
        obs.pose.x = obstacle.pose.x;
        obs.pose.y = obstacle.pose.y;
        obs.pose.theta = obstacle.pose.theta;

        for (auto &point : obstacle.shape.points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            obs.shape.points.push_back(p);
        }
        response->obstacles.push_back(obs);
    }

    for (auto &cargo : world_.cargos)
    {
        uvs_message::msg::DscpCargo c;
        c.name = cargo.name;

        for (auto &point : cargo.shape.points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            c.shape.points.push_back(p);
        }
        for (auto &anchor : cargo.anchors)
        {
            geometry_msgs::msg::Point32 p;
            p.x = anchor.x;
            p.y = anchor.y;
            p.z = 0;
            c.anchors.push_back(p);
        }
        response->cargos.push_back(c);
    }

    for (auto &agent : world_.agents)
    {
        uvs_message::msg::DscpAgent a;
        a.name = agent.name;

        for (auto &point : agent.shape.points)
        {
            geometry_msgs::msg::Point32 p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0;
            a.shape.points.push_back(p);
        }
        for (auto &anchor : agent.anchors)
        {
            geometry_msgs::msg::Point32 p;
            p.x = anchor.x;
            p.y = anchor.y;
            p.z = 0;
            a.anchors.push_back(p);
        }
        response->agents.push_back(a);
    }

}

// void UVSMapServer::uvQueryElementCallback(const std::shared_ptr<rmw_request_id_t> request_header,
//                                           const std::shared_ptr<uvs_message::srv::UvQueryElement::Request> request,
//                                           std::shared_ptr<uvs_message::srv::UvQueryElement::Response> response)
// {
    
//     for (auto &qname : request->name_list)
//     {
//         if (uv_opt_poses_.find(qname) == uv_opt_poses_.end())
//         {
//             // RCLCPP_ERROR(get_logger(), "Failed to find uv_opt_pose: %s", qname.c_str());
//             response->pose_list.clear();
//             return;
//         }
//         response->pose_list.push_back(uv_opt_poses_[qname]);
//     }
// }

// void UVSMapServer::uvQueryMapCallback(const std::shared_ptr<rmw_request_id_t> request_header,
//     const std::shared_ptr<uvs_message::srv::UvQueryMap::Request> request, 
//     std::shared_ptr<uvs_message::srv::UvQueryMap::Response> response)
// {
//     if (request->name != world_.name)
//     {
//         // RCLCPP_ERROR(get_logger(), "Failed to find world: %s", request->name.c_str());
//         return;
//     }
//     map_static_.header.stamp = now();
//     response->map = map_static_;
//     for (auto &fps : cargo_shape_)
//     {
//         auto it_pose_opt = uv_opt_poses_.find(fps.first);
//         if (it_pose_opt == uv_opt_poses_.end())
//         {
//             // RCLCPP_ERROR(get_logger(), "Failed to find uv_opt_pose: %s", fps.first.c_str());
//             continue;
//         }
//         geometry_msgs::msg::Pose pose_opt = it_pose_opt->second;
//         Pose2D pose(pose_opt.position.x, pose_opt.position.y, tf2::getYaw(pose_opt.orientation));

//         auto& fp = fps.second;
//         // for (auto& point : fp.second)
//         // {
//         //     response->map.data[point.y * map_static_.info.width + point.x] = 0;
//         // }
//         Polygon2D temp = fp.first.transform(pose);
//         fp.second = getFootprint(temp, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(map_static_.info.origin.position.x, map_static_.info.origin.position.y));
//         for (auto& point : fp.second)
//         {
//             response->map.data[point.y * map_static_.info.width + point.x] = 75;
//         }

//     }
// }

void UVSMapServer::uvOptPoseListCallback(const uvs_message::msg::UvOptPoseList::SharedPtr msg)
{
    // uv_opt_poses_.clear();

    for (auto &uv_opt_pose : msg->pose_list)
    {
        uv_opt_poses_[uv_opt_pose.name] = uv_opt_pose.pose;
    }
}

void UVSMapServer::timerCallback()
{
    nav_msgs::msg::OccupancyGrid map = map_static_;
    map.header.stamp = now();
    for (auto &fps : cargo_shape_)
    {
        auto it_pose_opt = uv_opt_poses_.find(fps.first);
        if (it_pose_opt == uv_opt_poses_.end())
        {
            // RCLCPP_ERROR(get_logger(), "Failed to find uv_opt_pose: %s", fps.first.c_str());
            continue;
        }
        geometry_msgs::msg::Pose pose_opt = it_pose_opt->second;
        Pose2D pose(pose_opt.position.x, pose_opt.position.y, tf2::getYaw(pose_opt.orientation));

        auto& fp = fps.second;
        // for (auto& point : fp.second)
        // {
        //     response->map.data[point.y * map_static_.info.width + point.x] = 0;
        // }
        Polygon2D temp = fp.first.transform(pose);
        fp.second = getFootprint(temp, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(map_static_.info.origin.position.x, map_static_.info.origin.position.y));
        for (auto& point : fp.second)
        {
            map.data[point.y * map_static_.info.width + point.x] = 75;
        }
    }
    for (auto &a : world_.agents)
    {
        auto it_pose_opt = uv_opt_poses_.find(a.name);
        if (it_pose_opt == uv_opt_poses_.end())
        {
            // RCLCPP_ERROR(get_logger(), "Failed to find uv_opt_pose: %s", a.name.c_str());
            continue;
        }
        geometry_msgs::msg::Pose pose_opt = it_pose_opt->second;
        Pose2D pose(pose_opt.position.x, pose_opt.position.y, tf2::getYaw(pose_opt.orientation));

        auto& fp = a.shape;
        // for (auto& point : fp.second)
        // {
        //     response->map.data[point.y * map_static_.info.width + point.x] = 0;
        // }
        Polygon2D temp = fp.transform(pose);
        std::vector<Point2I> footprint = getFootprint(temp, world_.ground.resolutionX, world_.ground.resolutionY, Point2D(map_static_.info.origin.position.x, map_static_.info.origin.position.y));
        for (auto& point : footprint)
        {
            map.data[point.y * map_static_.info.width + point.x] = 50;
        }
    }
    map_static_pub_->publish(map);
}

std::vector<Point2I> UVSMapServer::getFootprint(const Polygon2D &shape, const double &resolutionX, const double &resolutionY, const Point2D &origin)
{
    return shape.distribute(resolutionX, resolutionY, origin);
}


