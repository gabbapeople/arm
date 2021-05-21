
#include "arm_driver.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_driver");

    ArmDriver ad(0.01);

    ros::spin();
    return 0;
}
