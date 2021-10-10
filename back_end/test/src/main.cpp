#include <iostream>
#include <eigen3/Eigen/Dense>
#include <ceres/ceres.h>
#include <vector>
#include "ceres_test.hpp"

using namespace std;

int main() {
    Eigen::Vector3d a{1, 1, 1};
    Eigen::Vector3d b{2, 2, 2};
    Eigen::Vector3d c{3, 3, 3};

    vector<Eigen::Vector3d> x_v;
    vector<Eigen::Vector3d> y_v;

    double a_[3] = {0, 0, 0};
    double b_[3] = {0, 0, 0};
    double c_[3] = {0, 0, 0};

    ceres::Problem problem;
    problem.AddParameterBlock(a_, 3);
    problem.AddParameterBlock(b_, 3);
    problem.AddParameterBlock(c_, 3);

    for(int i = 1; i < 100; i++) {
        Eigen::Vector3d x{i * 0.1, i * 0.1, i * 0.1};
        x_v.push_back(x);
        Eigen::Vector3d y;
        y(0) = a.dot(x) + 2 * b.dot(x) - 3 * c.dot(x);
        y(1) = b.dot(x) + 2 * a.dot(x) - 3 * c.dot(x);
        y(2) = c.dot(x) + 2 * a.dot(x) - 3 * b.dot(x);
        y_v.push_back(y);

        CeresTest *c_t = new CeresTest(x, y);
        problem.AddResidualBlock(c_t, NULL, a_, b_, c_);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.trust_region_strategy_type = ceres::DOGLEG;
    options.minimizer_progress_to_stdout=true;
    options.max_num_iterations = 20;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    cout<<summary.BriefReport()<<endl;

    cout<<a_[0]<<"  "<<a_[1]<<"  "<<a_[2]<<endl;
    cout<<b_[0]<<"  "<<b_[1]<<"  "<<b_[2]<<endl;
    cout<<c_[0]<<"  "<<c_[1]<<"  "<<c_[2]<<endl;

    return 0;
}