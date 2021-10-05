#include <iostream>
#include "generate_synthetic_data.hpp"

using namespace std;

int main(int argc, char **argv) {
    SyntheticData *s_d = new SyntheticData(20, 100, 20, 100, 90);
    s_d->generate_pose();
    s_d->generate_imu_data();
    s_d->mid_point_intergration();
    s_d->generate_image_data();
    s_d->check_with_fundmental_matrix();


}