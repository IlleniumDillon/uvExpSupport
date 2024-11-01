#include "load_world.hpp"

#include <rclcpp/rclcpp.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ament_index_cpp/get_package_prefix.hpp"

#include "tinyxml2.h"

void loadWorld(std::string name, DSCP_World &scene)
{
    /**
     * clear the scene
     */
    scene.name = "";
    scene.agents.clear();
    scene.cargos.clear();
    scene.obstacles.clear();
    scene.ground.shape.points.clear();
    /**
     * get the world file path
     */
    std::string package_name = "uvs_mapserver";
    std::string package_share_directory = ament_index_cpp::get_package_share_directory(package_name);
    std::string world_path = package_share_directory + "/world/" + name + ".world";
    /**
     * load the world file
     */
    tinyxml2::XMLDocument doc;
    // tinyxml2::XMLElement *pElement;
    if (doc.LoadFile(world_path.c_str()) != tinyxml2::XML_SUCCESS)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to load world file: %s", world_path.c_str());
        doc.Clear();
        return;
    }
    /**
     * get world name
     */
    tinyxml2::XMLElement *pWorld = doc.FirstChildElement("world");
    if (pWorld == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find world element");
        doc.Clear();
        return;
    }
    std::cout << "world name: " << pWorld->Attribute("name") << std::endl;
    scene.name = pWorld->Attribute("name");
    /**
     * get ground
     */
    tinyxml2::XMLElement *pGround = pWorld->FirstChildElement("ground");
    if (pGround == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find ground element");
        doc.Clear();
        return;
    }
    std::cout << "ground" << std::endl;
    tinyxml2::XMLElement *pGPoint = pGround->FirstChildElement("vertex");
    if (pGPoint == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find vertex element");
        doc.Clear();
        return;
    }
    for (; pGPoint != nullptr; pGPoint = pGPoint->NextSiblingElement("vertex"))
    {
        Point2D point;
        point.x = pGPoint->DoubleAttribute("x");
        point.y = pGPoint->DoubleAttribute("y");
        scene.ground.shape.points.push_back(point);
        std::cout << "vertex: " << point.x << ", " << point.y << std::endl;
    }
    tinyxml2::XMLElement *pGOrigin = pGround->FirstChildElement("origin");
    if (pGOrigin == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find origin element");
        doc.Clear();
        return;
    }
    scene.ground.origin.x = pGOrigin->DoubleAttribute("x");
    scene.ground.origin.y = pGOrigin->DoubleAttribute("y");
    std::cout << "origin: " << scene.ground.origin.x << ", " << scene.ground.origin.y << std::endl;
    /**
     * get agents
     */
    tinyxml2::XMLElement *pAgent = pWorld->FirstChildElement("agent");
    if (pAgent == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find agent element");
        doc.Clear();
        return;
    }
    for (; pAgent != nullptr; pAgent = pAgent->NextSiblingElement("agent"))
    {
        DSCP_Agent agent;
        agent.name = pAgent->Attribute("name");
        agent.max_speed = pAgent->DoubleAttribute("max_speed");
        agent.min_speed = pAgent->DoubleAttribute("min_speed");
        agent.max_angular_speed = pAgent->DoubleAttribute("max_angular_speed");
        agent.min_angular_speed = pAgent->DoubleAttribute("min_angular_speed");
        std::cout << "agent: " << agent.name << std::endl;
        tinyxml2::XMLElement *pPoint = pAgent->FirstChildElement("vertex");
        if (pPoint == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find vertex element");
            doc.Clear();
            return;
        }
        for (; pPoint != nullptr; pPoint = pPoint->NextSiblingElement("vertex"))
        {
            Point2D point;
            point.x = pPoint->DoubleAttribute("x");
            point.y = pPoint->DoubleAttribute("y");
            agent.shape.points.push_back(point);
            std::cout << "vertex: " << point.x << ", " << point.y << std::endl;
        }
        tinyxml2::XMLElement *pAnchor = pAgent->FirstChildElement("anchor");
        if (pAnchor == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find anchor element");
            doc.Clear();
            return;
        }
        for (; pAnchor != nullptr; pAnchor = pAnchor->NextSiblingElement("anchor"))
        {
            Point2D point;
            point.x = pAnchor->DoubleAttribute("x");
            point.y = pAnchor->DoubleAttribute("y");
            agent.anchors.push_back(point);
            std::cout << "anchor: " << point.x << ", " << point.y << std::endl;
        }
        scene.agents.push_back(agent);
    }
    /**
     * get cargos
     */
    tinyxml2::XMLElement *pCargo = pWorld->FirstChildElement("cargo");
    if (pCargo == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find cargo element");
        doc.Clear();
        return;
    }
    for (; pCargo != nullptr; pCargo = pCargo->NextSiblingElement("cargo"))
    {
        DSCP_Cargo cargo;
        cargo.name = pCargo->Attribute("name");
        std::cout << "cargo: " << cargo.name << std::endl;
        tinyxml2::XMLElement *pPoint = pCargo->FirstChildElement("vertex");
        if (pPoint == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find vertex element");
            doc.Clear();
            return;
        }
        for (; pPoint != nullptr; pPoint = pPoint->NextSiblingElement("vertex"))
        {
            Point2D point;
            point.x = pPoint->DoubleAttribute("x");
            point.y = pPoint->DoubleAttribute("y");
            cargo.shape.points.push_back(point);
            std::cout << "vertex: " << point.x << ", " << point.y << std::endl;
        }
        tinyxml2::XMLElement *pAnchor = pCargo->FirstChildElement("anchor");
        if (pAnchor == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find anchor element");
            doc.Clear();
            return;
        }
        for (; pAnchor != nullptr; pAnchor = pAnchor->NextSiblingElement("anchor"))
        {
            Point2D point;
            point.x = pAnchor->DoubleAttribute("x");
            point.y = pAnchor->DoubleAttribute("y");
            cargo.anchors.push_back(point);
            std::cout << "anchor: " << point.x << ", " << point.y << std::endl;
        }
        scene.cargos.push_back(cargo);
    }
    /**
     * get obstacles
     */
    tinyxml2::XMLElement *pObstacle = pWorld->FirstChildElement("obstacle");
    if (pObstacle == nullptr)
    {
        RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find obstacle element");
        doc.Clear();
        return;
    }
    for (; pObstacle != nullptr; pObstacle = pObstacle->NextSiblingElement("obstacle"))
    {
        DSCP_Obstacle obstacle;
        std::cout << "obstacle" << std::endl;
        tinyxml2::XMLElement *pPoint = pObstacle->FirstChildElement("vertex");
        if (pPoint == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find vertex element");
            doc.Clear();
            return;
        }
        for (; pPoint != nullptr; pPoint = pPoint->NextSiblingElement("vertex"))
        {
            Point2D point;
            point.x = pPoint->DoubleAttribute("x");
            point.y = pPoint->DoubleAttribute("y");
            obstacle.shape.points.push_back(point);
            std::cout << "vertex: " << point.x << ", " << point.y << std::endl;
        }
        
        tinyxml2::XMLElement *pPose = pObstacle->FirstChildElement("pose");
        if (pPose == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("uvs_tools"), "Failed to find origin element");
            doc.Clear();
            return;
        }
        obstacle.pose.x = pPose->DoubleAttribute("x");
        obstacle.pose.y = pPose->DoubleAttribute("y");
        obstacle.pose.theta = pPose->DoubleAttribute("theta");
        std::cout << "pose: " << obstacle.pose.x << ", " << obstacle.pose.y << ", " << obstacle.pose.theta << std::endl;

        // Calculate footprint based on shape and pose
        for (const auto& point : obstacle.shape.points)
        {
            Point2D transformed_point;
            transformed_point.x = point.x * cos(obstacle.pose.theta) - point.y * sin(obstacle.pose.theta) + obstacle.pose.x;
            transformed_point.y = point.x * sin(obstacle.pose.theta) + point.y * cos(obstacle.pose.theta) + obstacle.pose.y;
            obstacle.footprint.points.push_back(transformed_point);
            std::cout << "footprint: " << transformed_point.x << ", " << transformed_point.y << std::endl;
        }

        scene.obstacles.push_back(obstacle);
    }
}