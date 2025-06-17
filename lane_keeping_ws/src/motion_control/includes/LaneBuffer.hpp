#pragma once

#include <deque>
#include <vector>

constexpr int ROAD_WIDTH = 256;

class LaneBuffer
{
    public:
        LaneBuffer(size_t max_size);
        ~LaneBuffer() = default;

        virtual std::vector<double> getLastLeft();
        virtual std::vector<double> getLastRight();

        std::vector<double> estimateOtherLane(std::vector<double>& coefs,
                                              bool from_left);

        virtual void addCoeffs(const std::vector<double>& left_coefs,
                               const std::vector<double>& right_coefs);

        virtual bool hasLeftLane();
        virtual bool hasRightLane();

        size_t getLeftSize();
        size_t getRightSize();

    private:
        size_t max_size_;
        std::deque<std::vector<double>> left_lane_;
        std::deque<std::vector<double>> right_lane_;
};
