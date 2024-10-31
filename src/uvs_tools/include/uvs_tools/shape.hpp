#ifndef UVS_TOOLS_SHAPE_HPP
#define UVS_TOOLS_SHAPE_HPP

#include <iostream>
#include <vector>

class Point2D
{
public:
    double x;
    double y;
};

class Pose2D
{
public:
    double x;
    double y;
    double theta;
};

class Polygon2D
{
public:
    std::vector<Point2D> points;
};

#endif // UVS_TOOLS_SHAPE_HPP