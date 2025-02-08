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

    void innovateKalman(const double& delta_time_s);

    void updateKalman(const Point& measurement);

    bool hasBallBeenKicked(const BallDetection& new_ball_detection, double delta_time_s);
private:
    /**
     * Return the point closest to from that is not inside of a robot
     * (although if two robots are adjacent or inside each other this is not guaranteed)
     */
    std::optional<Point> intersectRobotsWithLine(const Point& from, const Point& to, const Team& friendly_team, const Team& enemy_team);

    /**
     *
     * @param friendly_team
     * @param enemy_team
     * @return a pair containing the robot who is dribbling and the local offset from the center of the robot to the ball
     */
    std::optional<std::pair<Robot, Vector>> getDribblerContainingBall(const Team& friendly_team, const Team& enemy_team);

    Timestamp last_update_time_s;

    double max_friction_acceleration;

    bool last_collided_with_bot = false; // true if the last tick had a predicted ball colliding with ball
    bool last_invisible = false;         // true if the last tick was invisible

    enum Mode {
        KALMAN,
        PUSH,
        DRIBBLE,
        AIR
    };

    RobotId bot_in_question_id = -1;

    std::optional<Vector> dribble_offset;

    unsigned int ticks_dribbling = 0;

    Mode current_mode;
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
     *
     * Control Space:
     * [
     *  x acceleration
     *  y acceleration
     * ]
     */
    KalmanFilter<4, 2, 2> kalman_filter;

};
