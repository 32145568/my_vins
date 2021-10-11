#include "visual_measurement_factor.hpp"

VisualFactor::VisualFactor() {
    ;
}

VisualFactor::VisualFactor(BackEndParameter *b_p_, Eigen::Vector3d observation_i_, Eigen::Vector3d observation_j_) {
    b_p = b_p_;
    observation_i = observation_i_;
    observation_j = observation_j_;
    info_matrix = Eigen::Matrix2d::Zero();
    info_matrix(0, 0) = b_p->focal_length / 1.5;
    info_matrix(1, 1) = b_p->focal_length / 1.5;
    //info_matrix = Eigen::Matrix2d::Identity();
}

Eigen::Matrix3d get_skew_symmetric_matrix_(Eigen::Vector3d &v) {
    Eigen::Matrix3d s_matrix = Eigen::Matrix3d::Zero();

    s_matrix(0, 1) = -v(2);
    s_matrix(1, 0) = v(2);
    s_matrix(0, 2) = v(1);
    s_matrix(2, 0) = -v(1);
    s_matrix(1, 2) = -v(0);
    s_matrix(2, 1) = v(0);

    return s_matrix;
}

bool VisualFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
//bool VisualFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) {
    Eigen::Vector3d translation_1(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation_1(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d translation_2(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond rotation_2(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d translation_ex(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond rotation_ex(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Matrix3d rotation_m_1;
    Eigen::Matrix3d rotation_m_2;
    Eigen::Matrix3d rotation_m_ex;
    Eigen::Vector3d observation_i_ = observation_i;
    Eigen::Vector3d observation_j_ = observation_j;

    double inverse_dep_l = parameters[3][0];
    //cout<<inverse_dep_l<<endl;
    Eigen::Vector3d cal_observation_j;

    rotation_m_1 = rotation_1;
    rotation_m_2 = rotation_2;
    rotation_m_ex = rotation_ex;

    observation_i_ = (1.0 / inverse_dep_l) * observation_i_;
    cal_observation_j = rotation_ex * observation_i_ + translation_ex;
    cal_observation_j = rotation_1 * cal_observation_j + translation_1;
    cal_observation_j = rotation_2.inverse() * (cal_observation_j - translation_2);
    cal_observation_j = rotation_ex.inverse() * (cal_observation_j - translation_ex);

    residuals[0] = cal_observation_j(0) / cal_observation_j(2) - observation_j_(0) / observation_j_(2);
    residuals[1] = cal_observation_j(1) / cal_observation_j(2) - observation_j_(1) / observation_j_(2); 

    /*Eigen::Matrix<double, 2, 7> j_matrix_1;
    Eigen::Matrix<double, 2, 7> j_matrix_2;
    Eigen::Matrix<double, 2, 7> j_matrix_3;
    Eigen::Vector2d j_matrix_4;*/

    if(jacobians) {
        Eigen::Matrix<double, 2, 3> jacobian_matrix_0;
        jacobian_matrix_0 = Eigen::Matrix<double, 2, 3>::Zero();
        jacobian_matrix_0(0, 0) = 1.0 / cal_observation_j(2);
        jacobian_matrix_0(0, 2) = -cal_observation_j(0) / cal_observation_j(2) / cal_observation_j(2);
        jacobian_matrix_0(1, 1) = 1.0 / cal_observation_j(2);
        jacobian_matrix_0(1, 2) = -cal_observation_j(1) / cal_observation_j(2) / cal_observation_j(2);

        if(jacobians[0]) {
            Eigen::Vector3d p;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 3, 7> m;
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_matrix_1(jacobians[0]);

            m = Eigen::Matrix<double, 3, 7>::Zero();
            m.block<3, 3>(0, 0) = rotation_m_ex.transpose() * rotation_m_2.transpose();
            p = rotation_m_ex * observation_i_ + translation_ex;
            sk_m = get_skew_symmetric_matrix_(p);
            sk_m = - rotation_m_ex.transpose() * rotation_m_2.transpose() * rotation_m_1 * sk_m;
            m.block<3, 3>(0, 3) = sk_m;
            jacobian_matrix_1 = jacobian_matrix_0 * m;
            //j_matrix_1 = jacobian_matrix_1;
        }

        if(jacobians[1]) {
            Eigen::Vector3d p;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 3, 7> m;
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_matrix_2(jacobians[1]);
            
            m = Eigen::Matrix<double, 3, 7>::Zero();
            m.block<3, 3>(0, 0) = - rotation_m_ex.transpose() * rotation_m_2.transpose();
            p = rotation_m_ex * observation_i_ + translation_ex;
            p = rotation_m_1 * p + translation_1;
            p = rotation_m_2.transpose() * (p - translation_2);
            sk_m = get_skew_symmetric_matrix_(p);
            sk_m = rotation_m_ex.transpose() * sk_m;
            m.block<3, 3>(0, 3) = sk_m;
            jacobian_matrix_2 = jacobian_matrix_0 * m;
            //j_matrix_2 = jacobian_matrix_2;
        }

        if(jacobians[2]) {
            Eigen::Vector3d p;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 3, 7> m;
            Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> jacobian_matrix_3(jacobians[2]);

            m = Eigen::Matrix<double, 3, 7>::Zero();
            sk_m = rotation_m_ex.transpose() * rotation_m_2.transpose() * rotation_m_1 - rotation_m_ex.transpose();
            m.block<3, 3>(0, 0) = sk_m;
            sk_m = get_skew_symmetric_matrix_(observation_i_);
            sk_m = -rotation_m_ex.transpose() * rotation_m_2.transpose() * rotation_m_1 * rotation_m_ex * sk_m;
            m.block<3, 3>(0, 3) = sk_m;   
            p = rotation_m_ex.transpose() * rotation_m_2.transpose() * rotation_m_1 * rotation_m_ex * observation_i_;
            sk_m = get_skew_symmetric_matrix_(p);
            m.block<3, 3>(0, 3) += sk_m;   
            p = rotation_m_1 * translation_ex + translation_1 - translation_2;
            p = rotation_m_2.transpose() * p - translation_ex;
            p = rotation_m_ex.transpose() * p;
            sk_m = get_skew_symmetric_matrix_(p);
            m.block<3, 3>(0, 3) += sk_m;   
            jacobian_matrix_3 = jacobian_matrix_0 * m;
            //j_matrix_3 = jacobian_matrix_3;
        }

        if(jacobians[3]) {
            Eigen::Map<Eigen::Vector2d> jacobian_matrix_4(jacobians[3]);
            Eigen::Vector3d p;

            p = - observation_i_ / inverse_dep_l;
            p = rotation_m_ex.transpose() * rotation_m_2.transpose() * rotation_m_1 * rotation_m_ex * p;
            jacobian_matrix_4 = jacobian_matrix_0 * p;
            //j_matrix_4 = jacobian_matrix_4;
        }
    }  

#if 0
    observation_i_ = inverse_dep_l * observation_i_;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double inv_dep_i = parameters[3][0];

    Eigen::Vector3d pts_camera_i = observation_i_ / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Vector2d residual;


    double dep_j = pts_camera_j.z();
    residual = (pts_camera_j / dep_j).head<2>() - observation_j_.head<2>();

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 2, 3> reduce(2, 3);

        reduce << 1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j),
            0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j);

        if (jacobians[0])
        {
            Eigen::Matrix<double, 2, 7> jacobian_pose_i;
            Eigen::Matrix<double, 3, 6> jaco_i;
            jaco_i.leftCols<3>() = ric.transpose() * Rj.transpose();
            jaco_i.rightCols<3>() = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();

            j_matrix_1 -= jacobian_pose_i;
            cout<<j_matrix_1.norm()<<endl;
            //cout<<jacobian_pose_i<<endl;
        }

        if (jacobians[1])
        {
            Eigen::Matrix<double, 2, 7> jacobian_pose_j;
            Eigen::Matrix<double, 3, 6> jaco_j;
            jaco_j.leftCols<3>() = ric.transpose() * -Rj.transpose();
            jaco_j.rightCols<3>() = ric.transpose() * Utility::skewSymmetric(pts_imu_j);

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
            j_matrix_2 -= jacobian_pose_j;
            cout<<j_matrix_2.norm()<<"  ";
        }

        if (jacobians[2])
        {
            Eigen::Matrix<double, 2, 7> jacobian_ex_pose;
            Eigen::Matrix<double, 3, 6> jaco_ex;
            jaco_ex.leftCols<3>() = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.rightCols<3>() = -tmp_r * Utility::skewSymmetric(pts_camera_i) + Utility::skewSymmetric(tmp_r * pts_camera_i) +
                                     Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
            j_matrix_3 -= jacobian_ex_pose;
            cout<<j_matrix_3.norm()<<"   ";
        }

        if (jacobians[3])
        {
            Eigen::Vector2d jacobian_feature;
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * observation_i_ * -1.0 / (inv_dep_i * inv_dep_i);
            j_matrix_4 -= jacobian_feature;
            cout<<j_matrix_4.norm()<<endl;
        }
    }
#endif

    return true;
}
