#include <cmath>
#include <limits>
#include <stdexcept>

typedef struct Vector {
  double x, y, z;
  Vector(double x=0, double y=0, double z=0) :
  x(x), y(y), z(z) {}
  Vector(const Vector& p) : 
  x(p.x), y(p.y), z(p.z) {}
  Vector operator+(const Vector& other) const {
    return Vector(x+other.x, y+other.y, z+other.z);
  } 
  Vector operator-(const Vector& other) const {
    return Vector(x-other.x, y-other.y, z-other.z);
  } 
  Vector operator*(const double& other) const {
    return Vector(x*other, y*other, z*other);
  } 
  double dot(const Vector& other) const {
    return x * other.x + y * other.y + z * other.z;
  }
  Vector cross(const Vector& other) const {
    return Vector(y * other.z - z * other.y,
                  z * other.x - x * other.z,
                  x * other.y - y * other.x);
  }
} Point;

class Line {
private:
  Point p_start_, p_end_;
  Vector v_line_;
public:
  Line(const Point& p_start, const Point& p_end) :
  p_start_(p_start), p_end_(p_end), v_line_(p_end_-p_start_) {
    if (std::fabs(p_start_.x - p_end_.x) < std::numeric_limits<double>::epsilon() &&
        std::fabs(p_start_.y - p_end_.y) < std::numeric_limits<double>::epsilon() &&
        std::fabs(p_start_.z - p_end_.z) < std::numeric_limits<double>::epsilon()) {
          throw std::invalid_argument("Error: The two points defining the line are identical!");
        }
  }
  double signed_distance_to_point(const Point& p) {
    Vector v_p_to_p_start = p - p_start_;
    Vector cross = v_line_.cross(v_p_to_p_start);
    return std::hypot(cross.x, cross.y, cross.z) / std::hypot(v_line_.x, v_line_.y, v_line_.z);
  }
  Point foot_to_point(const Point& p) {
    Vector v_p_to_p_start = p - p_start_;
    double dot = v_line_.dot(v_p_to_p_start);
    double t = dot / pow(std::hypot(v_line_.x, v_line_.y, v_line_.z),2);
    return Point(p_start_ + v_p_to_p_start * t);
  }
};