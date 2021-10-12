#include "imu_factor_v1.hpp"

bool PoseLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const {
    Eigen::Map<const Eigen::Vector3d> translation(x);
    Eigen::Map<const Eigen::Quaterniond> rotation(x + 3);
    Eigen::Vector3d delta_translation{delta[0], delta[1], delta[2]};
    Eigen::Quaterniond delta_rotation{1, 0.5 * delta[3], 0.5 * delta[4], 0.5 * delta[5]};

    Eigen::Map<Eigen::Vector3d> translation_plus_delta(x_plus_delta);
    Eigen::Map<Eigen::Quaterniond> rotation_plus_delta(x_plus_delta + 3);

    translation_plus_delta = translation + delta_translation;
    rotation_plus_delta = (rotation * delta_rotation).normalized();
    /*if(delta_translation.norm() > 0.5) {
        cout<<delta_translation<<endl;
    } */
    return true;
}

bool PoseLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const {
    Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian_matrix(jacobian);
    jacobian_matrix.topRows<6>().setIdentity();
    jacobian_matrix.bottomRows<1>().setZero();
    return true;
}

ImuIntergration::ImuIntergration(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_, Eigen::Vector3d acc_0_, Eigen::Vector3d gyr_0_, BackEndParameter *b_p_) {
    acc_bias = acc_bias_;
    gyr_bias = gyr_bias_;
    acc_0 = acc_0_;
    gyr_0 = gyr_0_;    
    b_p = b_p_;
    last_acc = acc_0 - acc_bias;
    last_gyr = gyr_0 - gyr_bias;
    temp_rotation.x() = 0;
    temp_rotation.y() = 0;
    temp_rotation.z() = 0;
    temp_rotation.w() = 1;
    temp_translation = Eigen::Vector3d::Zero();
    temp_velocity = Eigen::Vector3d::Zero();
    sum_time = 0;

    noise_matrix = Eigen::Matrix<double, 12, 12>::Zero();
    covariance_matrix = Eigen::Matrix<double, 9, 9>::Zero();
    jacobian_matrix = Eigen::Matrix<double, 15, 15>::Identity();

    noise_matrix.block<3, 3>(0, 0) = b_p->acc_n * b_p->acc_n * Eigen::Matrix<double, 3, 3>::Identity();
    noise_matrix.block<3, 3>(3, 3) = b_p->gyr_n * b_p->gyr_n * Eigen::Matrix<double, 3, 3>::Identity();
    noise_matrix.block<3, 3>(6, 6) = noise_matrix.block<3, 3>(0, 0);
    noise_matrix.block<3, 3>(9, 9) = noise_matrix.block<3, 3>(3, 3);
}

void ImuIntergration::add_state(Eigen::Vector3d acc, Eigen::Vector3d gyr, double delta_t) {
    delta_t_q.push_back(delta_t);
    acc_q.push_back(acc);
    gyr_q.push_back(gyr);
}

Eigen::Matrix3d get_skew_symmetric_matrix(Eigen::Vector3d &v) {
    Eigen::Matrix3d s_matrix = Eigen::Matrix3d::Zero();

    s_matrix(0, 1) = -v(2);
    s_matrix(1, 0) = v(2);
    s_matrix(0, 2) = v(1);
    s_matrix(2, 0) = -v(1);
    s_matrix(1, 2) = -v(0);
    s_matrix(2, 1) = v(0);

    return s_matrix;
}

void ImuIntergration::mid_intergration() {
    Eigen::Vector3d aver_gyr, aver_acc;
    Eigen::Matrix<double, 15, 15> matrix_F = Eigen::Matrix<double, 15, 15>::Zero();
    Eigen::Matrix<double, 9, 12> matrix_V = Eigen::Matrix<double, 9, 12>::Zero();
    Eigen::Matrix3d matrix_I = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d last_rotation;
    Eigen::Matrix3d current_rotation;

    aver_gyr = (last_gyr + temp_gyr) * 0.5;
    Eigen::Quaterniond delta_q;
    delta_q.w() = 1;
    delta_q.x() = 0.5 * temp_delta_t * aver_gyr(0);
    delta_q.y() = 0.5 * temp_delta_t * aver_gyr(1);
    delta_q.z() = 0.5 * temp_delta_t * aver_gyr(2);
    aver_acc = (temp_rotation * last_acc + temp_rotation * delta_q * temp_acc) * 0.5;
    last_rotation = temp_rotation;
    //cout<<last_rotation<<endl;
    temp_rotation = temp_rotation * delta_q;
    current_rotation = temp_rotation;
    temp_translation = temp_translation + temp_velocity * temp_delta_t + aver_acc * temp_delta_t * temp_delta_t * 0.5;
    temp_velocity = temp_velocity + aver_acc * temp_delta_t;
    sum_time += temp_delta_t; 

    Eigen::Matrix3d matrix_F_01, matrix_F_03, matrix_F_04;
    Eigen::Matrix3d matrix_F_11;
    Eigen::Matrix3d matrix_F_21, matrix_F_23, matrix_F_24;
    Eigen::Matrix3d skm_w_1 = get_skew_symmetric_matrix(last_gyr);
    Eigen::Matrix3d skm_w_2 = get_skew_symmetric_matrix(temp_gyr);
    Eigen::Matrix3d skm_a_1 = get_skew_symmetric_matrix(last_acc);
    Eigen::Matrix3d skm_a_2 = get_skew_symmetric_matrix(temp_acc);

    matrix_F_03 = -0.25 * (last_rotation + current_rotation) * temp_delta_t * temp_delta_t;
    matrix_F_11 = matrix_I - 0.5 * (skm_w_1 + skm_w_2) * temp_delta_t;
    matrix_F_21 = -0.5 * last_rotation * skm_a_1 * temp_delta_t - 0.5 * current_rotation * skm_a_2 * (matrix_I - 0.5 * (skm_w_1 + skm_w_2) * temp_delta_t) * temp_delta_t;
    matrix_F_23 = -0.5 * (last_rotation + current_rotation) * temp_delta_t;
    matrix_F_24 = 0.5 * current_rotation * skm_a_2 * temp_delta_t * temp_delta_t;
    matrix_F_01 = 0.5 * temp_delta_t * matrix_F_21;
    matrix_F_04 = 0.5 * temp_delta_t * matrix_F_24;

    //cout<<matrix_F_03<<endl;

    matrix_F.block<3, 3>(0, 0) = matrix_I;
    matrix_F.block<3, 3>(0, 3) = matrix_F_01;
    matrix_F.block<3, 3>(0, 6) = temp_delta_t * matrix_I;
    matrix_F.block<3, 3>(0, 9) = matrix_F_03;
    matrix_F.block<3, 3>(0, 12) = matrix_F_04;
    matrix_F.block<3, 3>(3, 3) = matrix_F_11;
    matrix_F.block<3, 3>(3, 12) = -1 * temp_delta_t * matrix_I;
    matrix_F.block<3, 3>(6, 3) = matrix_F_21;
    matrix_F.block<3, 3>(6, 6) = matrix_I;
    matrix_F.block<3, 3>(6, 9) = matrix_F_23;
    matrix_F.block<3, 3>(6, 12) = matrix_F_24;
    matrix_F.block<3, 3>(9, 9) = matrix_I;
    matrix_F.block<3, 3>(12, 12) = matrix_I;

    Eigen::Matrix3d matrix_V_00, matrix_V_01, matrix_V_02, matrix_V_03;
    Eigen::Matrix3d matrix_V_21, matrix_V_23;

    matrix_V_00 = -0.25 * last_rotation * temp_delta_t * temp_delta_t;
    matrix_V_02 = -0.25 * current_rotation * temp_delta_t * temp_delta_t;
    matrix_V_21 = 0.25 * current_rotation * skm_a_2 * temp_delta_t * temp_delta_t;
    matrix_V_23 = matrix_V_21;
    matrix_V_01 = 0.5 * temp_delta_t * matrix_V_21;
    matrix_V_03 = matrix_V_01;

    matrix_V.block<3, 3>(0, 0) = matrix_V_00;
    matrix_V.block<3, 3>(0, 3) = matrix_V_01;
    matrix_V.block<3, 3>(0, 6) = matrix_V_02;
    matrix_V.block<3, 3>(0, 9) = matrix_V_03;
    matrix_V.block<3, 3>(3, 3) = -0.5 * temp_delta_t * matrix_I;
    matrix_V.block<3, 3>(3, 9) = -0.5 * temp_delta_t * matrix_I;
    matrix_V.block<3, 3>(6, 0) = -0.5 * temp_delta_t * last_rotation;
    matrix_V.block<3, 3>(6, 3) = matrix_V_21;
    matrix_V.block<3, 3>(6, 6) = -0.5 * temp_delta_t * current_rotation;
    matrix_V.block<3, 3>(6, 9) = matrix_V_23;

    jacobian_matrix = matrix_F * jacobian_matrix;
    covariance_matrix = matrix_F.block<9, 9>(0, 0) * covariance_matrix * matrix_F.block<9, 9>(0, 0).transpose() + matrix_V * noise_matrix * matrix_V.transpose();
}

void ImuIntergration::propagate() {

    for(size_t i = 0; i < acc_q.size(); i++) {
        temp_acc = acc_q[i] - acc_bias;
        temp_gyr = gyr_q[i] - gyr_bias;
        temp_delta_t = delta_t_q[i];
        mid_intergration();

        last_acc = temp_acc;
        last_gyr = temp_gyr;        
    }
}

void ImuIntergration::re_propagete(Eigen::Vector3d acc_bias_, Eigen::Vector3d gyr_bias_) {
    acc_bias = acc_bias_;
    gyr_bias = gyr_bias_;

    last_acc = acc_0;
    last_gyr = gyr_0;
  
    temp_rotation.x() = 0;
    temp_rotation.y() = 0;
    temp_rotation.z() = 0;
    temp_rotation.w() = 1;
    temp_translation = Eigen::Vector3d::Zero();
    sum_time = 0;

    propagate();
}

Eigen::Matrix<double, 4, 4> get_left_multiply_matrix(Eigen::Quaterniond q) {
    Eigen::Matrix<double, 4, 4> left_multiply_matrix;
    double q4 = q.w();
    Eigen::Vector3d q_{q.x(), q.y(), q.z()};
    Eigen::Matrix3d sk_matrix = get_skew_symmetric_matrix(q_);

    left_multiply_matrix.block<3, 3>(0, 0) = q4 * Eigen::Matrix3d::Identity() + sk_matrix;
    left_multiply_matrix.block<3, 1>(0, 3) = q_;
    left_multiply_matrix.block<1, 3>(3, 0) = -q_.transpose();
    left_multiply_matrix(3, 3) = q4;

    return left_multiply_matrix;
}

Eigen::Matrix<double, 4, 4> get_right_multiply_matrix(Eigen::Quaterniond q) {
    Eigen::Matrix<double, 4, 4> right_multiply_matrix;
    double q4 = q.w();
    Eigen::Vector3d q_{q.x(), q.y(), q.z()};
    Eigen::Matrix3d sk_matrix = get_skew_symmetric_matrix(q_);

    right_multiply_matrix.block<3, 3>(0, 0) = q4 * Eigen::Matrix3d::Identity() - sk_matrix;
    right_multiply_matrix.block<3, 1>(0, 3) = q_;
    right_multiply_matrix.block<1, 3>(3, 0) = -q_.transpose();
    right_multiply_matrix(3, 3) = q4;

    return right_multiply_matrix;
}

Eigen::Vector3d get_delta_q(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
    q2 = q1.inverse() * q2;
    Eigen::Vector3d delta_q;
    delta_q(0) = 2 * (q2.x() / q2.w());
    delta_q(1) = 2 * (q2.y() / q2.w());
    delta_q(2) = 2 * (q2.z() / q2.w());
    return delta_q;
}

ImuFactor::ImuFactor(ImuIntergration *i_p_) {
    i_p = i_p_;
}

bool ImuFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
//bool ImuFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) {

    Eigen::Vector3d translation_1(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond rotation_1(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d translation_2(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond rotation_2(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Matrix3d rotation_matrix_1;
    Eigen::Matrix3d rotation_matrix_2; 
    rotation_matrix_1 = rotation_1;
    rotation_matrix_2 = rotation_2;

    Eigen::Vector3d velocity_1(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d velocity_2(parameters[3][0], parameters[3][1], parameters[3][2]);

    Eigen::Vector3d acc_bias(parameters[4][0], parameters[4][1], parameters[4][2]);
    Eigen::Vector3d gyr_bias(parameters[4][3], parameters[4][4], parameters[4][5]);

    Eigen::Map<Eigen::Matrix<double, 9, 1>> residuals_e(residuals);
    Eigen::Matrix<double, 9, 9> info_matrix;
    info_matrix = Eigen::LLT<Eigen::Matrix<double, 9, 9>>(i_p->covariance_matrix.inverse()).matrixL().transpose();

    Eigen::Matrix3d jacobian_matrix_pba = i_p->jacobian_matrix.block<3, 3>(0, 9);
    Eigen::Matrix3d jacobian_matrix_pbg = i_p->jacobian_matrix.block<3, 3>(0, 12);
    Eigen::Matrix3d jacobian_matrix_qbg = i_p->jacobian_matrix.block<3, 3>(3, 12);
    Eigen::Matrix3d jacobian_matrix_vba = i_p->jacobian_matrix.block<3, 3>(6, 9);
    Eigen::Matrix3d jacobian_matrix_vbg = i_p->jacobian_matrix.block<3, 3>(6, 12);

    Eigen::Vector3d temp_translation_ = i_p->temp_translation;
    Eigen::Quaterniond temp_rotation_ = i_p->temp_rotation;
    Eigen::Vector3d temp_velocity_ = i_p->temp_velocity;
    double sum_time = i_p->sum_time;
    //info_matrix.setIdentity();

    temp_translation_ = temp_translation_ +  jacobian_matrix_pba * (acc_bias - i_p->acc_bias);
    temp_translation_ = temp_translation_ + jacobian_matrix_pbg * (gyr_bias - i_p->gyr_bias);
    Eigen::Vector3d delta_theta = jacobian_matrix_qbg * (gyr_bias - i_p->gyr_bias);
    Eigen::Quaterniond delta_rotation{1, 0.5 * delta_theta(0), 0.5 * delta_theta(1), 0.5 * delta_theta(2)};
    temp_rotation_ = temp_rotation_ * delta_rotation;
    temp_velocity_ = temp_velocity_ + jacobian_matrix_vba * (acc_bias - i_p->acc_bias);
    temp_velocity_ = temp_velocity_ + jacobian_matrix_vbg * (gyr_bias - i_p->gyr_bias);

    Eigen::Vector3d cal_translation = translation_2 - translation_1 + 0.5 * i_p->g_acc * sum_time * sum_time - velocity_1 * sum_time;
    cal_translation = rotation_1.inverse() * cal_translation;
    Eigen::Quaterniond cal_rotation = rotation_1.inverse() * rotation_2;
    Eigen::Vector3d cal_velocity = rotation_1.inverse() * (velocity_2 - velocity_1 + i_p->g_acc * sum_time);

    residuals_e.block<3, 1>(0, 0) = cal_translation - temp_translation_;
    residuals_e.block<3, 1>(3, 0) = get_delta_q(temp_rotation_, cal_rotation);
    residuals_e.block<3, 1>(6, 0) = cal_velocity - temp_velocity_;

    residuals_e = info_matrix * residuals_e;

    if(jacobians) {
        if(jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 9, 7, Eigen::RowMajor>> jacobian_matrix_1(jacobians[0]);
            jacobian_matrix_1 = Eigen::Matrix<double, 9, 7>::Zero();

            Eigen::Vector3d v;
            Eigen::Quaterniond q;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 4, 4> lm_matrix, rm_matrix;

            jacobian_matrix_1.block<3, 3>(0, 0) = - rotation_matrix_1.transpose();
            sk_m = get_skew_symmetric_matrix(cal_translation);
            jacobian_matrix_1.block<3, 3>(0, 3) = sk_m;
            q = rotation_1 * rotation_2.inverse();
            lm_matrix = get_left_multiply_matrix(q);
            rm_matrix = get_right_multiply_matrix(temp_rotation_);
            lm_matrix = -lm_matrix * rm_matrix;
            jacobian_matrix_1.block<3, 3>(3, 3) = lm_matrix.block<3, 3>(0, 0);
            sk_m = get_skew_symmetric_matrix(cal_velocity);
            jacobian_matrix_1.block<3, 3>(6, 3) = sk_m;
            jacobian_matrix_1 = info_matrix * jacobian_matrix_1;
        }

        if(jacobians[1]) {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_matrix_2(jacobians[1]);
            jacobian_matrix_2 = Eigen::Matrix<double, 9, 3>::Zero();

            jacobian_matrix_2.block<3, 3>(0, 0) = -rotation_matrix_1.transpose() * sum_time;
            jacobian_matrix_2.block<3, 3>(6, 0) = -rotation_matrix_1.transpose();
            jacobian_matrix_2 = info_matrix * jacobian_matrix_2;
        }

        if(jacobians[2]) {
            Eigen::Map<Eigen::Matrix<double, 9, 7, Eigen::RowMajor>> jacobian_matrix_3(jacobians[2]);
            jacobian_matrix_3 = Eigen::Matrix<double, 9, 7>::Zero();

            Eigen::Vector3d v;
            Eigen::Quaterniond q;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 4, 4> lm_matrix, rm_matrix;

            jacobian_matrix_3.block<3, 3>(0, 0) = rotation_matrix_1.transpose();
            q = temp_rotation_.inverse() * rotation_1.inverse() * rotation_2;
            lm_matrix = get_left_multiply_matrix(q);
            jacobian_matrix_3.block<3, 3>(3, 3) = lm_matrix.block<3, 3>(0, 0);
            jacobian_matrix_3 = info_matrix * jacobian_matrix_3;
        }

        if(jacobians[3]) {
            Eigen::Map<Eigen::Matrix<double, 9, 3, Eigen::RowMajor>> jacobian_matrix_4(jacobians[3]);
            jacobian_matrix_4 = Eigen::Matrix<double, 9, 3>::Zero();
            jacobian_matrix_4.block<3, 3>(6, 0) = rotation_matrix_1.transpose();
            jacobian_matrix_4 = info_matrix * jacobian_matrix_4;
        }

        if(jacobians[4]) {
            Eigen::Map<Eigen::Matrix<double, 9, 6, Eigen::RowMajor>> jacobian_matrix_5(jacobians[4]);
            jacobian_matrix_5 = Eigen::Matrix<double, 9, 6>::Zero();

            Eigen::Vector3d v;
            Eigen::Quaterniond q;
            Eigen::Matrix3d sk_m;
            Eigen::Matrix<double, 4, 4> lm_matrix, rm_matrix;

            jacobian_matrix_5.block<3, 3>(0, 0) = -jacobian_matrix_pba;
            jacobian_matrix_5.block<3, 3>(0, 3) = -jacobian_matrix_pbg;
            q = rotation_2.inverse() * rotation_1 * temp_rotation_;
            lm_matrix = get_left_multiply_matrix(q);
            jacobian_matrix_5.block<3, 3>(3, 3) = -lm_matrix.block<3, 3>(0, 0) * jacobian_matrix_qbg;
            jacobian_matrix_5.block<3, 3>(6, 0) = -jacobian_matrix_vba;
            jacobian_matrix_5.block<3, 3>(6, 3) = -jacobian_matrix_vbg;
            jacobian_matrix_5 = info_matrix * jacobian_matrix_5;
        }
    }

#if 0
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    if (jacobians)
    {
        int O_P = 0;
        int O_R = 3;
        int O_V = 6;
        int O_BA = 9;
        int O_BG = 12;

        if (jacobians[0])
        {
            Eigen::Matrix<double, 15, 7> jacobian_pose_i;
            jacobian_pose_i.setZero();

            jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(Qi.inverse() * (0.5 * g_acc * sum_time * sum_time + Pj - Pi - Vi * sum_time));

            Eigen::Quaterniond corrected_delta_q = temp_rotation * Utility::deltaQ(jacobian_matrix_qbg * (Bgi - gyr_bias));
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) * Utility::Qright(corrected_delta_q)).bottomRightCorner<3, 3>();
            jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (g_acc * sum_time + Vj - Vi));

            jacobian_pose_i = m_1 - jacobian_pose_i;
            cout<<m_1.norm()<<"   "<<jacobian_pose_i.norm()<<endl;
        }

        if (jacobians[1])
        {
            Eigen::Matrix<double, 15, 9> jacobian_speedbias_i;
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_time;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -jacobian_matrix_pba;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -jacobian_matrix_pbg;
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * temp_rotation).bottomRightCorner<3, 3>() * jacobian_matrix_qbg;
            jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -jacobian_matrix_vba;
            jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -jacobian_matrix_vbg;
            jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();
            jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

            jacobian_speedbias_i = jacobian_speedbias_i - m_2;
            cout<<m_2.norm()<<"   "<<jacobian_speedbias_i.norm()<<endl;
        }

        if (jacobians[2])
        {
            Eigen::Matrix<double, 15, 7> jacobian_pose_j;
            jacobian_pose_j.setZero();
            jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();
            Eigen::Quaterniond corrected_delta_q = temp_rotation * Utility::deltaQ(jacobian_matrix_qbg * (Bgi - gyr_bias));
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(corrected_delta_q.inverse() * Qi.inverse() * Qj).bottomRightCorner<3, 3>();

            jacobian_pose_j = jacobian_pose_j - m_3;
            cout<<m_3.norm()<<"   "<<jacobian_pose_j.norm()<<endl;
        }

        if (jacobians[3])
        {
            Eigen::Matrix<double, 15, 9> jacobian_speedbias_j;
            jacobian_speedbias_j.setZero();
            jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();
            jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();
            jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j = jacobian_speedbias_j - m_4;
            cout<<m_4.norm()<<"   "<<jacobian_speedbias_j.norm()<<endl;
        }
    }
#endif
    return true;
}