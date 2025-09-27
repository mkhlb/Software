#pragma once

#include <boost/circular_buffer.hpp>
#include <optional>

#include "software/geom/line.h"
#include "software/geom/point.h"
#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/kalman_filter.hpp"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"

/**
 * Given ball data from SSL Vision, filters and returns the position/velocity of the
 * "real" ball.
 *
 * This ball filter stores a buffer of previous SSL Vision detections, and uses linear
 * regression to find the path the ball is travelling on and estimate its position
 * and velocity. This buffer/regression system was chosen because it results in a
 * very stable output, particularly for the ball velocity. The data we receive isn't
 * perfect (which is why we have a filter). If we receive a noisy position that is off
 * the ball's current trajectory, it will have minimal impact. This means that as
 * the ball is travelling, this filter will return a very steady velocity vector.
 * This is important because small deviations in velocity orientation can have large
 * effects when the AI tries to predict the future position of the ball. For example,
 * consistently receiving a pass relies on the ball's velocity being very stable,
 * otherwise the robot would "jiggle" back and forth as the estimated receiver position
 * would keep changing.
 */
class BallFilter
{
   public:

    double BALL_DECELERATION_M_PER_S_2 = 0.5;

    /**
     * Creates a new Ball Filter
     */
    explicit BallFilter();

    /**
     * Update the filter with the new ball detection data, and returns the new
     * estimated state of the ball given the new data
     *
     * @param new_ball_detections A list of new Ball detections
     * @param filter_area The area within which the ball filter will work. Any detections
     * outside of this area will be ignored.
     *
     * @return The new ball based on the estimated state of the ball given the new data.
     * If a filtered result cannot be calculated, returns std::nullopt
     */
    Ball estimateBallState(
        const std::vector<BallDetection>& new_ball_detections,
        const Rectangle& filter_area);

   private:

    void processDetections(std::vector<BallDetection> new_ball_detections, const Rectangle &filter_area);

    void processDetection(const BallDetection &detection, const Rectangle &filter_area);

    void processVisibleDetection(const BallDetection &detection);

    void processInvisibleDetection(const BallDetection &detection);

    Ball getBall();

    Timestamp latest_detection_timestamp_;

    /**
     * Kalman filter for ball has state vector with 4 variables:
     * x position
     * x velocity
     * y position
     * y velocity
     *
     * measurement vector with 2 variables
     * x position
     * y position
     *
     * and no control input.
     *
     * The state innovation is simple kinematic motion assuming constant deceleration for the velocity.
     * Collisions and other non-linear cases are handled with heuristics, overriding the kalman filter's prediction.
     * In the future, an unscented kalman filter could be used to move all of these non-linear cases to the filter.
     */
    KalmanFilter<4, 2, 0> kalman_filter_;
};
