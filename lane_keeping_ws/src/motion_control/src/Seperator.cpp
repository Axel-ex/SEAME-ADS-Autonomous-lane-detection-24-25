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

    // convert y = mx + b ==> Ax + By + C = D
    // A = -m  B = 1  C = -b
    for (auto& pt : lane)
    {
        d = -1 * coef[2] * pt.x + pt.y - coef[0];
        if (d > 0)
        {
            x.push_back(pt.second);
            y.push_back(pt.first);
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

    // convert y = mx + b ==> Ax + By + C = D
    // A = -m  B = 1  C = -b
    for (auto& pt : lane)
    {
        d = -1 * coef[2] * pt.x + pt.y - coef[0];
        if (d > 0)
        {
            x.push_back(pt.second);
            y.push_back(pt.first);
        }
    }
}