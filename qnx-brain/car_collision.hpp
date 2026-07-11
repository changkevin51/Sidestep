#ifndef V2V_CAR_COLLISION_HPP
#define V2V_CAR_COLLISION_HPP

#include <string>

namespace car_collision {

// All local coordinates use metres, with +x pointing east and +y north.
struct Vec2 {
    double x;
    double y;

    Vec2();
    Vec2(double x_value, double y_value);

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(double scalar) const;
    Vec2 operator/(double scalar) const;
    Vec2& operator+=(const Vec2& other);

    double dot(const Vec2& other) const;
    double lengthSquared() const;
    double length() const;
    Vec2 normalized() const;
};

Vec2 operator*(double scalar, const Vec2& vector);

// DMS degrees are a non-negative magnitude. The hemisphere supplies the sign.
// N/S coordinates are validated as latitudes and E/W as longitudes.
struct DMS {
    double degrees;
    double minutes;
    double seconds;
    char hemisphere;

    DMS();
    DMS(double degrees_value,
        double minutes_value,
        double seconds_value,
        char hemisphere_value);
};

struct GeographicPoint {
    double latitude_deg;
    double longitude_deg;

    GeographicPoint();
    GeographicPoint(double latitude_value, double longitude_value);
};

// Non-throwing conversion for data received from an untrusted source. On
// failure, decimal_degrees is left unchanged. error_message may be null.
bool tryDmsToDecimal(const DMS& dms,
                     double& decimal_degrees,
                     std::string* error_message = 0);

// Throws std::invalid_argument when the DMS fields or hemisphere are invalid.
double dmsToDecimal(const DMS& dms);
double dmsToDecimal(double degrees,
                    double minutes,
                    double seconds,
                    char hemisphere);

// Equirectangular local tangent-plane approximation using an Earth radius of
// 6,371,000 m. The result is relative to reference and expressed as (east,
// north). Throws std::invalid_argument for non-finite/out-of-range coordinates.
Vec2 latLonToLocalMeters(double latitude_deg,
                        double longitude_deg,
                        double reference_latitude_deg,
                        double reference_longitude_deg);
Vec2 latLonToLocalMeters(const GeographicPoint& point,
                        const GeographicPoint& reference);

// Short spelling retained for call sites where the unit is already apparent.
Vec2 latLonToLocal(double latitude_deg,
                   double longitude_deg,
                   double reference_latitude_deg,
                   double reference_longitude_deg);

// Vehicle heading is in degrees: 0 = north, 90 = east, increasing clockwise.
// Vehicle dimensions must be positive and speed must be non-negative.
struct Car {
    std::string id;
    Vec2 center;
    double length_m;
    double width_m;
    double heading_deg;
    double speed_mps;

    Car();
    Car(const std::string& car_id,
        const Vec2& center_position,
        double length_metres,
        double width_metres,
        double heading_degrees,
        double speed_metres_per_second);

    Vec2 forward() const;
    Vec2 velocity() const;
    Vec2 positionAt(double time_seconds) const;
};

struct OrientedBoundingBox {
    Vec2 center;
    Vec2 forward_axis;
    Vec2 right_axis;
    double half_length_m;
    double half_width_m;
};

// Returns false and optionally describes the first invalid field.
bool validateCar(const Car& car, std::string* error_message = 0);

OrientedBoundingBox makeOrientedBoundingBox(const Car& car,
                                             double time_seconds = 0.0);

// Touching edges count as overlap. Invalid cars cause std::invalid_argument.
bool obbOverlap(const OrientedBoundingBox& first,
                const OrientedBoundingBox& second);
bool obbOverlap(const Car& first, const Car& second);
bool carsOverlapAtTime(const Car& first,
                       const Car& second,
                       double time_seconds);

struct CollisionResult {
    bool will_collide;
    double ttc_seconds;

    CollisionResult();
    CollisionResult(bool collision_expected, double collision_time_seconds);
};

// Performs a fixed-step SAT look-ahead. Each step interval is swept so a brief
// overlap cannot fall between samples. No collision is represented by
// will_collide == false and +infinity TTC.
CollisionResult sweepForCollision(const Car& first,
                                  const Car& second,
                                  double max_time_seconds = 10.0,
                                  double step_seconds = 0.05);

// Convenience form: returns +infinity when there is no collision in the
// requested window. This makes std::isfinite(timeToCollision(...)) a simple
// test for a predicted collision.
double timeToCollision(const Car& first,
                       const Car& second,
                       double max_time_seconds = 10.0,
                       double step_seconds = 0.05);

}  // namespace car_collision

#endif  // V2V_CAR_COLLISION_HPP
