#include "point_to_line_distance.hpp"
#include <string>
#include <iostream>

bool test(double x_s, double y_s, double z_s,
          double x_e, double y_e, double z_e,
          double x_p, double y_p, double z_p,
          double expected_distance) {
            Point p_start(x_s,y_s,z_s);
            Point p_end(x_e,y_e,z_e);
            Point p(x_p,y_p,z_p);
            Line l(p_start,p_end);
            double distance = l.signed_distance_to_point(p);
            std::cout<<"expected_distance="<<expected_distance<<std::endl;
            std::cout<<"calculated_distance="<<distance<<std::endl;
            Point foot = l.foot_to_point(p);
            std::cout<<"foot="<<foot.x<<" "<<foot.y<<std::endl;
            return expected_distance == distance;
          }

int main() {
    std::cout<<"Hello Test"<<std::endl;
    double x_s=0, y_s=0, z_s=0;
    double x_e=5, y_e=0, z_e=0;
    double x_p=2, y_p=3, z_p=4;
    double expected_distance=5;
    std::string passed = test(x_s, y_s, z_s, \
                              x_e, y_e, z_e, \
                              x_p, y_p, z_p, \
                              expected_distance)?"true":"false";
    std::cout<<"test passed? "<<passed<<std::endl;

    x_s=0;  y_s=0; z_s=0;
    x_e=10; y_e=0; z_e=0;
    x_p=3;  y_p=4; z_p=0;
    expected_distance=4;
    passed = test(x_s, y_s, z_s, \
                  x_e, y_e, z_e, \
                  x_p, y_p, z_p, \
                  expected_distance)?"true":"false";
    std::cout<<"test passed? "<<passed<<std::endl;
}