#include "software/sensor_fusion/filter/ball_filter.h"

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <vector>
#include "software/sensor_fusion/filter/kalman_filter.hpp"

#include "shared/constants.h"
#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/contains.h"
#include "software/math/math_functions.h"


BallFilter::BallFilter(): kalman_filter_()
{
    kalman_filter_.x << 0, 0,
                        0, 0;

    kalman_filter_.P << 1000, 1000,
                        1000, 1000;
}

// get the estimated current position of the ball
Ball BallFilter::estimateBallState(
    const std::vector<BallDetection> &new_ball_detections, const Rectangle &filter_area)
{
    processDetections(new_ball_detections, filter_area);
    return getBall();
}

// processes detections in order
void BallFilter::processDetections(std::vector<BallDetection> new_ball_detections,
                                   const Rectangle &filter_area)
{
    // Sort the detections in increasing order before processing. This places the oldest
    // detections (with the smallest timestamp) at the front of the list
    std::sort(new_ball_detections.begin(), new_ball_detections.end());

    for (const auto &detection : new_ball_detections)
    {
        Duration time_diff =
                detection.timestamp - latest_detection_timestamp;

        // Ignore any data from the past, and any data that is as old as the oldest
        // data in the buffer since it provides no additional value. This also
        // prevents division by 0 when calculating the estimated velocity
        if (time_diff.toSeconds() <= 0)
        {
            continue;
        }

        processDetection(detection, filter_area);
    }
}

void BallFilter::processDetection(const BallDetection &detection, const Rectangle &filter_area)
{
    if (detection.position.has_value())
    {
        Point position = detection.position.value();
        // Remove any detections outside the filter area
        if (!contains(filter_area, position))
        {
            return;
        }

        processVisibleDetection(detection);
    }
    else
    {
        processInvisibleDetection(detection);
    }

    latest_detection_timestamp = detection.timestamp;
}

void BallFilter::processVisibleDetection(const BallDetection &detection)
{

}

void BallFilter::processInvisibleDetection(const BallDetection &detection)
{

}

Ball BallFilter::getBall()
{

}