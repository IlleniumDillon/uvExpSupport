#ifndef UVS_TOOLS_SHAPE_HPP
#define UVS_TOOLS_SHAPE_HPP

#include <iostream>
#include <vector>
#include <cmath>

template <typename T>
class Point2T
{
public:
    double x;
    double y;
public:
    Point2T() : x(0), y(0) {}
    Point2T(T x, T y) : x(x), y(y) {}
    Point2T(const Point2T<T> &p) : x(p.x), y(p.y) {}
    Point2T<T> &operator=(const Point2T<T> &p)
    {
        x = p.x;
        y = p.y;
        return *this;
    }

    Point2T<T> operator+(const Point2T<T> &p) const
    {
        return Point2T<T>(x + p.x, y + p.y);
    }

    Point2T<T> operator-(const Point2T<T> &p) const
    {
        return Point2T<T>(x - p.x, y - p.y);
    }

    Point2T<T> operator*(double s) const
    {
        return Point2T<T>(x * s, y * s);
    }

    Point2T<T> operator/(double s) const
    {
        return Point2T<T>(x / s, y / s);
    }

    double dot(const Point2T<T> &p) const
    {
        return x * p.x + y * p.y;
    }

    double cross(const Point2T<T> &p) const
    {
        return x * p.y - y * p.x;
    }

    double norm() const
    {
        return sqrt(x * x + y * y);
    }

    Point2T<T> normalize() const
    {
        return *this / norm();
    }

    double distance(const Point2T<T> &p) const
    {
        return (*this - p).norm();
    }

};

typedef Point2T<double> Point2D;
typedef Point2T<int> Point2I;

class Pose2D
{
public:
    double x;
    double y;
    double theta;
public:
    Pose2D() : x(0), y(0), theta(0) {}
    Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    Pose2D(const Pose2D &p) : x(p.x), y(p.y), theta(p.theta) {}
    Pose2D &operator=(const Pose2D &p)
    {
        x = p.x;
        y = p.y;
        theta = p.theta;
        return *this;
    }

    double distance(const Pose2D &p) const
    {
        return Point2D(x, y).distance(Point2D(p.x, p.y));
    }

};

class Polygon2D
{
public:
    std::vector<Point2D> points;
public:
    std::vector<Point2I> distribute(double resolutionX, double resolutionY, const Point2D &origin) const;
};

#endif // UVS_TOOLS_SHAPE_HPP