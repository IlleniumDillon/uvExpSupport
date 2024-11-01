#ifndef UVS_MAPSERVER_LOAD_WORLD_HPP
#define UVS_MAPSERVER_LOAD_WORLD_HPP

#include <iostream>
#include <string>
#include <vector>

#include "shape.hpp"

class DSCP_Agent
{
public:
    std::string name;
    double max_speed;
    double min_speed;
    double max_angular_speed;
    double min_angular_speed;
    Polygon2D shape;
    std::vector<Point2D> anchors;
};
class DSCP_Cargo
{
public:
    std::string name;
    Polygon2D shape;
    std::vector<Point2D> anchors;
};
class DSCP_Ground
{
public:
    Point2D origin;
    Polygon2D shape;
    double resolutionX;
    double resolutionY;
};
class DSCP_Obstacle
{
public:
    Polygon2D shape;
    Pose2D pose;
    Polygon2D footprint;
};

class DSCP_World
{
public:
    std::string name;
    DSCP_Ground ground;
    std::vector<DSCP_Agent> agents;
    std::vector<DSCP_Cargo> cargos;
    std::vector<DSCP_Obstacle> obstacles;
};

void loadWorld(std::string name, DSCP_World& scene);

#endif // UVS_MAPSERVER_LOAD_WORLD_HPP