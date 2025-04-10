#include "./includes/Seperator.hpp"

void MotionControlNode::separateAndOrderCoordinates(
    const std::vector<Point32>& points, std::vector<double>& x,
    std::vector<double>& y)
    
std::vector<Point32> seperateRight(std::vector<std::pair<double, double>> lane)
{
    std::vector<double> x, y;

    for (const auto& pt : lane)
    {
        x.push_back(pt.second);
        y.push_back(pt.first);
    }

    std::vector<double> coefs = calculate(x.data(), y.data(), 1, x.size());
    float d;

    x.clear();
    y.clear();
    auto min_it = std::min_element(lane.begin(), lane.end(), 
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            return a.first < b.first;
        });
    // convert y = mx + b ==> Ax + By + C = D
    // A = -m  B = 1  C = -b
    if (-1 * coef[2] * min_it.first + min_it.second - coef[0] < 0)
    {
        for (auto& pt : lane)
        {
            d = -1 * coef[2] * pt.first + pt.second - coef[0];
            if (d < 0)
            {
                x.push_back(pt.second);
                y.push_back(pt.first);
            }
        }
    }
    else
    {
        for (auto& pt : lane)
        {
            d = -1 * coef[2] * pt.first + pt.second - coef[0];
            if (d > 0)
            {
                x.push_back(pt.second);
                y.push_back(pt.first);
            }
        }
    }
}

std::vector<Point32> seperateLeft(std::vector<std::pair<double, double>> lane)
{
    std::vector<double> x, y;

    for (const auto& pt : lane)
    {
        x.push_back(pt.second);
        y.push_back(pt.first);
    }

    std::vector<double> coefs = calculate(x.data(), y.data(), 1, x.size());
    float d;

    x.clear();
    y.clear();
    auto max_it = std::max_element(lane.begin(), lane.end(), 
        [](const std::pair<double, double>& a, const std::pair<double, double>& b) {
            return a.first < b.first;
        });
    // convert y = mx + b ==> Ax + By + C = D
    // A = -m  B = 1  C = -b
    if (-1 * coef[2] * max_it.first + max_it.second - coef[0] < 0)
    {
        for (auto& pt : lane)
        {
            d = -1 * coef[2] * pt.first + pt.second - coef[0];
            if (d < 0)
            {
                x.push_back(pt.second);
                y.push_back(pt.first);
            }
        }
    }
    else
    {
        for (auto& pt : lane)
        {
            d = -1 * coef[2] * pt.first + pt.second - coef[0];
            if (d > 0)
            {
                x.push_back(pt.second);
                y.push_back(pt.first);
            }
        }
    }
}