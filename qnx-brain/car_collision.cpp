#include "car_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace car_collision {
namespace {

const double kEarthRadiusMeters = 6371000.0;
const double kPi = 3.1415926535897932384626433832795;
const double kSatEpsilon = 1e-9;

double degreesToRadians(double degrees) {
    return degrees * kPi / 180.0;
}

bool isFinite(double value) {
    return std::isfinite(value) != 0;
}

void setError(std::string* destination, const std::string& message) {
    if (destination != 0) {
        *destination = message;
    }
}

char uppercaseAscii(char value) {
    if (value >= 'a' && value <= 'z') {
        return static_cast<char>(value - ('a' - 'A'));
    }
    return value;
}

void validateGeographicPoint(double latitude_deg,
                             double longitude_deg,
                             const char* label) {
    if (!isFinite(latitude_deg) || !isFinite(longitude_deg)) {
        throw std::invalid_argument(std::string(label) +
                                    " latitude/longitude must be finite");
    }
    if (latitude_deg < -90.0 || latitude_deg > 90.0) {
        throw std::invalid_argument(std::string(label) +
                                    " latitude must be in [-90, 90]");
    }
    if (longitude_deg < -180.0 || longitude_deg > 180.0) {
        throw std::invalid_argument(std::string(label) +
                                    " longitude must be in [-180, 180]");
    }
}

double normalizedLongitudeDeltaRadians(double longitude_delta_deg) {
    double radians = degreesToRadians(longitude_delta_deg);
    const double full_turn = 2.0 * kPi;
    radians = std::fmod(radians + kPi, full_turn);
    if (radians < 0.0) {
        radians += full_turn;
    }
    return radians - kPi;
}

void requireValidCar(const Car& car) {
    std::string error;
    if (!validateCar(car, &error)) {
        throw std::invalid_argument(error);
    }
}

double projectionRadius(const OrientedBoundingBox& box, const Vec2& axis) {
    return box.half_length_m * std::fabs(box.forward_axis.dot(axis)) +
           box.half_width_m * std::fabs(box.right_axis.dot(axis));
}

bool overlapsOnAxis(const OrientedBoundingBox& first,
                    const OrientedBoundingBox& second,
                    const Vec2& axis) {
    const double axis_length = axis.length();
    if (axis_length <= kSatEpsilon) {
        // makeOrientedBoundingBox always produces unit axes. Treating a
        // degenerate external axis as non-separating keeps this helper safe.
        return true;
    }

    const Vec2 unit_axis = axis / axis_length;
    const double center_distance =
        std::fabs((second.center - first.center).dot(unit_axis));
    const double combined_radius = projectionRadius(first, unit_axis) +
                                   projectionRadius(second, unit_axis);
    return center_distance <= combined_radius + kSatEpsilon;
}

// Finds the exact overlap entry within one fixed-step sweep interval. Sampling
// only the interval endpoint could miss a very brief overlap when vehicles are
// moving quickly, so the same four SAT axes are swept using relative velocity.
bool firstOverlapInInterval(const Car& first,
                            const Car& second,
                            double interval_start,
                            double interval_end,
                            double& overlap_time) {
    const OrientedBoundingBox first_box =
        makeOrientedBoundingBox(first, 0.0);
    const OrientedBoundingBox second_box =
        makeOrientedBoundingBox(second, 0.0);
    const Vec2 axes[4] = {first_box.forward_axis,
                          first_box.right_axis,
                          second_box.forward_axis,
                          second_box.right_axis};
    const Vec2 initial_delta = second.center - first.center;
    const Vec2 relative_velocity = second.velocity() - first.velocity();

    double entry_time = interval_start;
    double exit_time = interval_end;
    for (int index = 0; index < 4; ++index) {
        const Vec2 axis = axes[index].normalized();
        const double radius = projectionRadius(first_box, axis) +
                              projectionRadius(second_box, axis);
        const double initial_projection = initial_delta.dot(axis);
        const double velocity_projection = relative_velocity.dot(axis);

        if (std::fabs(velocity_projection) <= kSatEpsilon) {
            if (std::fabs(initial_projection) > radius + kSatEpsilon) {
                return false;
            }
            continue;
        }

        double axis_entry = (-radius - initial_projection) /
                            velocity_projection;
        double axis_exit = (radius - initial_projection) /
                           velocity_projection;
        if (axis_entry > axis_exit) {
            std::swap(axis_entry, axis_exit);
        }

        entry_time = std::max(entry_time, axis_entry);
        exit_time = std::min(exit_time, axis_exit);
        if (entry_time > exit_time + kSatEpsilon) {
            return false;
        }
    }

    if (exit_time < interval_start - kSatEpsilon ||
        entry_time > interval_end + kSatEpsilon) {
        return false;
    }
    overlap_time = std::min(interval_end,
                            std::max(interval_start, entry_time));
    return true;
}

}  // namespace

Vec2::Vec2() : x(0.0), y(0.0) {}

Vec2::Vec2(double x_value, double y_value) : x(x_value), y(y_value) {}

Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator*(double scalar) const {
    return Vec2(x * scalar, y * scalar);
}

Vec2 Vec2::operator/(double scalar) const {
    if (std::fabs(scalar) <= kSatEpsilon) {
        throw std::invalid_argument("cannot divide Vec2 by zero");
    }
    return Vec2(x / scalar, y / scalar);
}

Vec2& Vec2::operator+=(const Vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

double Vec2::dot(const Vec2& other) const {
    return x * other.x + y * other.y;
}

double Vec2::lengthSquared() const {
    return dot(*this);
}

double Vec2::length() const {
    return std::sqrt(lengthSquared());
}

Vec2 Vec2::normalized() const {
    const double magnitude = length();
    if (magnitude <= kSatEpsilon) {
        return Vec2();
    }
    return *this / magnitude;
}

Vec2 operator*(double scalar, const Vec2& vector) {
    return vector * scalar;
}

DMS::DMS() : degrees(0.0), minutes(0.0), seconds(0.0), hemisphere('N') {}

DMS::DMS(double degrees_value,
         double minutes_value,
         double seconds_value,
         char hemisphere_value)
    : degrees(degrees_value),
      minutes(minutes_value),
      seconds(seconds_value),
      hemisphere(hemisphere_value) {}

GeographicPoint::GeographicPoint()
    : latitude_deg(0.0), longitude_deg(0.0) {}

GeographicPoint::GeographicPoint(double latitude_value, double longitude_value)
    : latitude_deg(latitude_value), longitude_deg(longitude_value) {}

bool tryDmsToDecimal(const DMS& dms,
                     double& decimal_degrees,
                     std::string* error_message) {
    if (!isFinite(dms.degrees) || !isFinite(dms.minutes) ||
        !isFinite(dms.seconds)) {
        setError(error_message, "DMS components must be finite numbers");
        return false;
    }

    if (dms.degrees < 0.0) {
        setError(error_message,
                 "DMS degrees must be non-negative; use the hemisphere for sign");
        return false;
    }
    if (dms.minutes < 0.0 || dms.minutes >= 60.0) {
        setError(error_message, "DMS minutes must be in [0, 60)");
        return false;
    }
    if (dms.seconds < 0.0 || dms.seconds >= 60.0) {
        setError(error_message, "DMS seconds must be in [0, 60)");
        return false;
    }

    const char hemisphere = uppercaseAscii(dms.hemisphere);
    const bool is_latitude = hemisphere == 'N' || hemisphere == 'S';
    const bool is_longitude = hemisphere == 'E' || hemisphere == 'W';
    if (!is_latitude && !is_longitude) {
        setError(error_message, "DMS hemisphere must be N, S, E, or W");
        return false;
    }

    const double maximum_degrees = is_latitude ? 90.0 : 180.0;
    const double magnitude = dms.degrees + dms.minutes / 60.0 +
                             dms.seconds / 3600.0;
    if (magnitude > maximum_degrees) {
        setError(error_message,
                 is_latitude
                     ? "latitude DMS magnitude cannot exceed 90 degrees"
                     : "longitude DMS magnitude cannot exceed 180 degrees");
        return false;
    }

    const bool negative = hemisphere == 'S' || hemisphere == 'W';
    decimal_degrees = negative ? -magnitude : magnitude;
    if (error_message != 0) {
        error_message->clear();
    }
    return true;
}

double dmsToDecimal(const DMS& dms) {
    double decimal_degrees = 0.0;
    std::string error;
    if (!tryDmsToDecimal(dms, decimal_degrees, &error)) {
        throw std::invalid_argument(error);
    }
    return decimal_degrees;
}

double dmsToDecimal(double degrees,
                    double minutes,
                    double seconds,
                    char hemisphere) {
    return dmsToDecimal(DMS(degrees, minutes, seconds, hemisphere));
}

Vec2 latLonToLocalMeters(double latitude_deg,
                        double longitude_deg,
                        double reference_latitude_deg,
                        double reference_longitude_deg) {
    validateGeographicPoint(latitude_deg, longitude_deg, "point");
    validateGeographicPoint(reference_latitude_deg,
                            reference_longitude_deg,
                            "reference");

    const double latitude_delta =
        degreesToRadians(latitude_deg - reference_latitude_deg);
    const double longitude_delta =
        normalizedLongitudeDeltaRadians(longitude_deg -
                                        reference_longitude_deg);
    const double reference_latitude_radians =
        degreesToRadians(reference_latitude_deg);

    const double east_metres = kEarthRadiusMeters * longitude_delta *
                               std::cos(reference_latitude_radians);
    const double north_metres = kEarthRadiusMeters * latitude_delta;
    return Vec2(east_metres, north_metres);
}

Vec2 latLonToLocalMeters(const GeographicPoint& point,
                        const GeographicPoint& reference) {
    return latLonToLocalMeters(point.latitude_deg,
                              point.longitude_deg,
                              reference.latitude_deg,
                              reference.longitude_deg);
}

Vec2 latLonToLocal(double latitude_deg,
                   double longitude_deg,
                   double reference_latitude_deg,
                   double reference_longitude_deg) {
    return latLonToLocalMeters(latitude_deg,
                              longitude_deg,
                              reference_latitude_deg,
                              reference_longitude_deg);
}

Car::Car()
    : id(),
      center(),
      length_m(0.0),
      width_m(0.0),
      heading_deg(0.0),
      speed_mps(0.0) {}

Car::Car(const std::string& car_id,
         const Vec2& center_position,
         double length_metres,
         double width_metres,
         double heading_degrees,
         double speed_metres_per_second)
    : id(car_id),
      center(center_position),
      length_m(length_metres),
      width_m(width_metres),
      heading_deg(heading_degrees),
      speed_mps(speed_metres_per_second) {}

Vec2 Car::forward() const {
    const double heading_radians =
        degreesToRadians(std::fmod(heading_deg, 360.0));
    return Vec2(std::sin(heading_radians), std::cos(heading_radians));
}

Vec2 Car::velocity() const {
    return forward() * speed_mps;
}

Vec2 Car::positionAt(double time_seconds) const {
    if (!isFinite(time_seconds) || time_seconds < 0.0) {
        throw std::invalid_argument("projection time must be finite and non-negative");
    }
    return center + velocity() * time_seconds;
}

bool validateCar(const Car& car, std::string* error_message) {
    if (!isFinite(car.center.x) || !isFinite(car.center.y)) {
        setError(error_message, "car center coordinates must be finite");
        return false;
    }
    if (!isFinite(car.length_m) || car.length_m <= 0.0) {
        setError(error_message, "car length must be finite and greater than zero");
        return false;
    }
    if (!isFinite(car.width_m) || car.width_m <= 0.0) {
        setError(error_message, "car width must be finite and greater than zero");
        return false;
    }
    if (!isFinite(car.heading_deg)) {
        setError(error_message, "car heading must be finite");
        return false;
    }
    if (!isFinite(car.speed_mps) || car.speed_mps < 0.0) {
        setError(error_message, "car speed must be finite and non-negative");
        return false;
    }

    if (error_message != 0) {
        error_message->clear();
    }
    return true;
}

OrientedBoundingBox makeOrientedBoundingBox(const Car& car,
                                             double time_seconds) {
    requireValidCar(car);
    if (!isFinite(time_seconds) || time_seconds < 0.0) {
        throw std::invalid_argument("OBB projection time must be finite and non-negative");
    }

    const Vec2 forward_axis = car.forward();
    OrientedBoundingBox box;
    box.center = car.positionAt(time_seconds);
    box.forward_axis = forward_axis;
    box.right_axis = Vec2(forward_axis.y, -forward_axis.x);
    box.half_length_m = car.length_m * 0.5;
    box.half_width_m = car.width_m * 0.5;
    return box;
}

bool obbOverlap(const OrientedBoundingBox& first,
                const OrientedBoundingBox& second) {
    const Vec2 axes[4] = {first.forward_axis,
                          first.right_axis,
                          second.forward_axis,
                          second.right_axis};
    for (int index = 0; index < 4; ++index) {
        if (!overlapsOnAxis(first, second, axes[index])) {
            return false;
        }
    }
    return true;
}

bool obbOverlap(const Car& first, const Car& second) {
    return carsOverlapAtTime(first, second, 0.0);
}

bool carsOverlapAtTime(const Car& first,
                       const Car& second,
                       double time_seconds) {
    return obbOverlap(makeOrientedBoundingBox(first, time_seconds),
                      makeOrientedBoundingBox(second, time_seconds));
}

CollisionResult::CollisionResult()
    : will_collide(false),
      ttc_seconds(std::numeric_limits<double>::infinity()) {}

CollisionResult::CollisionResult(bool collision_expected,
                                 double collision_time_seconds)
    : will_collide(collision_expected),
      ttc_seconds(collision_time_seconds) {}

CollisionResult sweepForCollision(const Car& first,
                                  const Car& second,
                                  double max_time_seconds,
                                  double step_seconds) {
    requireValidCar(first);
    requireValidCar(second);

    if (!isFinite(max_time_seconds) || max_time_seconds < 0.0) {
        throw std::invalid_argument(
            "maximum collision search time must be finite and non-negative");
    }
    if (!isFinite(step_seconds) || step_seconds <= 0.0) {
        throw std::invalid_argument(
            "collision search step must be finite and greater than zero");
    }

    if (carsOverlapAtTime(first, second, 0.0)) {
        return CollisionResult(true, 0.0);
    }

    double previous_time = 0.0;
    double sample_time = std::min(step_seconds, max_time_seconds);
    while (sample_time > previous_time &&
           sample_time <= max_time_seconds + kSatEpsilon) {
        double overlap_time = 0.0;
        if (firstOverlapInInterval(first,
                                   second,
                                   previous_time,
                                   sample_time,
                                   overlap_time)) {
            return CollisionResult(true, overlap_time);
        }

        previous_time = sample_time;
        if (previous_time >= max_time_seconds) {
            break;
        }
        sample_time = std::min(previous_time + step_seconds,
                               max_time_seconds);
    }

    return CollisionResult();
}

double timeToCollision(const Car& first,
                       const Car& second,
                       double max_time_seconds,
                       double step_seconds) {
    return sweepForCollision(first,
                             second,
                             max_time_seconds,
                             step_seconds)
        .ttc_seconds;
}

}  // namespace car_collision
