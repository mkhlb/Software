#include <optional>

#include "software/geom/rectangle.h"
#include "software/sensor_fusion/filter/vision_detection.h"
#include "software/time/timestamp.h"
#include "software/world/ball.h"
#include "software/world/team.h"
#include "software/sensor_fusion/filter/kalman_filter.h"

class BetterBallFilter {
public:
    BetterBallFilter(double ball_friction_acceleration, double process_noise_variance);

    /**
     * Prunes ball detections if they are outside of filter area, and checks if ball if flying, then either calls the function to process a new visible, invisible, or flying detection.
     * @param new_ball_detections
     * @param filter_area
     * @param friendly_team
     * @param enemy_team
     * @return
     */
    std::optional<Ball> estimateBallState(
            const std::vector<BallDetection>& new_ball_detections,
            const Rectangle& filter_area,
            const Team& friendly_team,
            const Team& enemy_team);

    /**
     *
     * @param new_ball_detection
     * @param friendly_team
     * @param enemy_team
     */
    void processVisibleNewDetection(const BallDetection& new_ball_detection, const Team& friendly_team, const Team& enemy_team);

    /**
     *
     * @param new_ball_detection
     * @param friendly_team
     * @param enemy_team
     */
    void processInvisibleNewDetection(const BallDetection& new_ball_detection, const Team& friendly_team, const Team& enemy_team);
private:

    enum Mode {
        KALMAN,
        PUSH,
        DRIBBLE,
        AIR
    };
    /**
     * 4-dimensional Kalman Filter
     *
     * State Space:
     * [
     *  x position
     *  x velocity
     *  y position
     *  y velocity
     * ]
     *
     * Measurement Space:
     * [
     *  x position
     *  y position
     * ]
     */
    KalmanFilter<4, 2, 0> kalman_filter;

};
