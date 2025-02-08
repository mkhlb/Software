
#include "better_ball_filter.h"

#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/distance.h"

void BetterBallFilter::processVisibleNewDetection(const BallDetection &new_ball_detection, const Team &friendly_team,
                                                  const Team &enemy_team) {
    if (last_invisible && (current_mode == Mode::DRIBBLE || current_mode == Mode::PUSH)) {
        current_mode = Mode::KALMAN;
    }

    // time since last detection
    double delta_time_s = (new_ball_detection.timestamp - last_update_time_s).toSeconds();

    if ((last_collided_with_bot && hasBallBeenKicked(new_ball_detection, delta_time_s)) || delta_time_s > 4) {
        // hard set ball position, velocity and skip innovation
        kalman_filter.x[0] = new_ball_detection.position->x();
        kalman_filter.x[2] = new_ball_detection.position->y();
        Vector velocity = (new_ball_detection.position.value() - Point(kalman_filter.x[0], kalman_filter.x[2])) / delta_time_s;
        kalman_filter.x[1] = velocity.x();
        kalman_filter.x[3] = velocity.y();
    } else {
        // do the innovation
        Point pos_before = Point(kalman_filter.x[0], kalman_filter.x[2]);
        innovateKalman(delta_time_s);
        // does the predicted state intersect a robot?
        std::optional<Point> intersection = intersectRobotsWithLine(pos_before, Point(kalman_filter.x[0], kalman_filter.x[2]), friendly_team, enemy_team);
        if (intersection.has_value()) {
            kalman_filter.x[0] = intersection->x();
            kalman_filter.x[2] = intersection->y();
        } else {
            updateKalman(new_ball_detection.position.value());
        }
    }
    auto dribbler = getDribblerContainingBall(friendly_team, enemy_team);
    if (dribbler.has_value()) {
        if (bot_in_question_id != dribbler->first.id()) {
            // new dribbler candidate!
            ticks_dribbling = 0;
            bot_in_question_id = dribbler->first.id();
        }
        ticks_dribbling++;
        if (ticks_dribbling > 5) {
            current_mode = Mode::DRIBBLE;
        }
    } else if (ticks_dribbling > 0) {
        ticks_dribbling--;
    }


}

void BetterBallFilter::innovateKalman(const double& delta_time_s) {
    Vector ball_velocity = Vector(kalman_filter.x[1], kalman_filter.x[3]);
    Vector a = (-ball_velocity).normalize(std::min(ball_velocity.length(), max_friction_acceleration * delta_time_s));
    // delta_time in seconds, a is friction acceleration
    Eigen::Matrix<double, 4, 4> A;
    A << 1.0, delta_time_s, 0.0, 0.0,
         0.0, 1.0,          0.0, 0.0,
         0.0, 0.0,          1.0, delta_time_s,
         0.0, 0.0,          0.0, 1.0;
    Eigen::Matrix<double, 2, 1> u = {a.x(), a.y()};
    kalman_filter.P = A;
    kalman_filter.predict(u);
}

void BetterBallFilter::updateKalman(const Point &measurement) {
    Eigen::Matrix<double, 2, 1> z;
    z << measurement.x(), measurement.y();
    kalman_filter.update(z);
}

bool BetterBallFilter::hasBallBeenKicked(const BallDetection& new_ball_detection, double delta_time_s) {

}

std::optional<Point> BetterBallFilter::intersectRobotsWithLine(const Point &from, const Point &to, const Team &friendly_team,
                                                const Team &enemy_team) {
    Segment segment = Segment(from, to);
    for (const Robot& r : friendly_team.getAllRobots()) {
        double p1 = distance(to, r.position());
        double p2 = distance(from, r.position());
        double max_distance = std::max(p1, p2);
        double min_distance = std::min(p1, p2);

        if (max_distance < ROBOT_MAX_RADIUS_METERS) {
            // both are inside the robot! Pop out and finish.
            auto bot_to_ball = from - r.position();
            return from + bot_to_ball.normalize(ROBOT_MAX_RADIUS_METERS - bot_to_ball.length());
        }

        if (min_distance > ROBOT_MAX_RADIUS_METERS + 0.8) {
            // robot is too far, ignore
            continue;
        }

        auto intersect = intersection(segment, Circle(r.position(), ROBOT_MAX_RADIUS_METERS));
        if (intersect.first.has_value()) {
            return intersect.first.value();
        }
    }

    for (const Robot& r : enemy_team.getAllRobots()) {
        double p1 = distance(to, r.position());
        double p2 = distance(from, r.position());
        double max_distance = std::max(p1, p2);
        double min_distance = std::min(p1, p2);

        if (max_distance < ROBOT_MAX_RADIUS_METERS) {
            // both are inside the robot! Pop out and finish.
            auto bot_to_ball = from - r.position();
            return from + bot_to_ball.normalize(ROBOT_MAX_RADIUS_METERS - bot_to_ball.length());
        }

        if (min_distance > ROBOT_MAX_RADIUS_METERS + 0.9) {
            // robot is too far, ignore
            continue;
        }

        auto intersect = intersection(segment, Circle(r.position(), ROBOT_MAX_RADIUS_METERS));
        if (intersect.first.has_value()) {
            return intersect.first.value();
        }
    }

    return std::nullopt;


}


