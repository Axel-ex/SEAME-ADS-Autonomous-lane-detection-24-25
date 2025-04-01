#include <LaneBuffer.hpp>

LaneBuffer::LaneBuffer(size_t max_size) : max_size_(max_size) {}

void LaneBuffer::addCoeffs(std::vector<double>& left_coefs,
                           std::vector<double>& right_coefs)
{
    if (right_coefs.size() >= 3)
        right_lane_.push_back(right_coefs);
    if (left_coefs.size() >= 3)
        left_lane_.push_back(left_coefs);

    if (right_lane_.size() >= max_size_)
        right_lane_.pop_front();
    if (left_lane_.size() >= max_size_)
        left_lane_.pop_front();
}

std::vector<double> LaneBuffer::getLastLeft() { return left_lane_.back(); }
std::vector<double> LaneBuffer::getLastRight() { return right_lane_.back(); }

bool LaneBuffer::hasLeftLane() { return !left_lane_.empty(); }

bool LaneBuffer::hasRightLane() { return !right_lane_.empty(); }

size_t LaneBuffer::getRightSize() { return right_lane_.size(); }
size_t LaneBuffer::getLeftSize() { return left_lane_.size(); }
