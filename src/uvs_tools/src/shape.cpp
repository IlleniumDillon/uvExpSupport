#include "shape.hpp"

#include <opencv2/opencv.hpp>

std::vector<Point2I> Polygon2D::distribute(double resolutionX, double resolutionY, const Point2D &origin) const
{
    std::vector<Point2I> points;

    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    std::vector<cv::Point2f> cv_points;
    for (auto &point : this->points)
    {
        cv_points.push_back(cv::Point2f(point.x, point.y));
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }

    int ori_x = origin.x / resolutionX;
    int ori_y = origin.y / resolutionY;

    int min_i = min_x / resolutionX;
    int min_j = min_y / resolutionY;
    int max_i = max_x / resolutionX;
    int max_j = max_y / resolutionY;

    for (int i = min_i; i <= max_i; i++)
    {
        for (int j = min_j; j <= max_j; j++)
        {
            if (cv::pointPolygonTest(cv_points, cv::Point2f(i*resolutionX, j*resolutionY), false) >= 0)
            {
                points.push_back(Point2I(i - ori_x, j - ori_y));
            }
        }
    }

    return points;
}