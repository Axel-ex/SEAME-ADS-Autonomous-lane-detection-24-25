#pragma once

#include <deque>
#include <vector>

class LaneBuffer
{
    public:
        LaneBuffer(size_t max_size);
        ~LaneBuffer() = default;

        std::vector<double> getLastLeft();
        std::vector<double> getLastRight();

        void addCoeffs(std::vector<double>& left_coefs,
                       std::vector<double>& right_coefs);

        bool hasLeftLane();
        bool hasRightLane();

        size_t getLeftSize();
        size_t getRightSize();

    private:
        size_t max_size_;
        std::deque<std::vector<double>> left_lane_;
        std::deque<std::vector<double>> right_lane_;
};
