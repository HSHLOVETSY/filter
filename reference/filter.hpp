/*
 * @Author: Shuai
 * @Date: 2025-07-31 16:10
 * @LastEditors: linzhuyue 
 * @LastEditTime: 2024-12-23 18:55:51
 * @FilePath: /cyan_lowlevel_sdk/src/hardware/rt_rc_interface.cpp
 * @Description: Writed In SH.
 * 
 * Copyright (c) 2024 by Rose, All Rights Reserved. 
 */

#ifndef EKF_FILTER_HPP
#define EKF_FILTER_HPP

#include <eigen3/Eigen/Dense>
template<typename T>
using Vec3 = Eigen::Matrix<T, 3, 1>;

class EKFFilter {
public:
    EKFFilter() {
        x.setZero();
        x.head<4>() << 1, 0, 0, 0;
        P.setIdentity();
        P *= 1e-4f;
        Q.setZero();
        Q_bias.setZero();
        double covariance_gyro_noise[3] = {1e-4f, 1e-4f, 1e-4f}; 
        double covariance_gyro_bias_noise[3] = {1e-4f, 1e-4f, 1e-4f}; 

        Q.diagonal() << covariance_gyro_noise[0], covariance_gyro_noise[1], covariance_gyro_noise[2];
        Q_bias.block<3, 3>(4, 4).diagonal() << covariance_gyro_bias_noise[0],
                                      covariance_gyro_bias_noise[1],
                                      covariance_gyro_bias_noise[2];        
                                      
        R.setIdentity();
        R *= 1e-6f;
    }

    void update(const Vec3<float>& acc, const Vec3<float>& gyro, float dt) {
        predict(gyro, dt);  
        correct(acc);   
    }

    Eigen::Quaternionf get_quaternion() const {
        return Eigen::Quaternionf(x(0), x(1), x(2), x(3)).normalized();
    }

    Eigen::VectorXf get_state() const {
        return x;
    }

private:
    Eigen::VectorXf x = Eigen::VectorXf::Zero(7);
    Eigen::VectorXf x_hat_minus = Eigen::VectorXf::Zero(7);
    Eigen::MatrixXf P = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf P_minus = Eigen::MatrixXf::Identity(7, 7);
    Eigen::MatrixXf Q = Eigen::Matrix3f::Zero();
    Eigen::MatrixXf Q_bias = Eigen::MatrixXf::Zero(7, 7);
    Eigen::MatrixXf R = Eigen::Matrix3f::Identity();

    void predict(const Vec3<float>& gyro, float dt) 
    {
        Eigen::Quaternionf q(x(0), x(1), x(2), x(3));
        Eigen::Quaternionf q_next = q; 
        Vec3<float> bg = x.tail<3>();
        Vec3<float> omega = gyro - bg;

        Eigen::Matrix4f omega_skew = get_omega_skew(omega);
        Eigen::Vector4f q_vec(q.w(), q.x(), q.y(), q.z());
        q_vec += 0.5f * omega_skew * q_vec * dt;

        q_next.w() = q_vec[0];
        q_next.x() = q_vec[1];
        q_next.y() = q_vec[2];
        q_next.z() = q_vec[3];
        q_next.normalize();

        x_hat_minus << q_next.w(), q_next.x(), q_next.y(), q_next.z(), bg(0), bg(1), bg(2);
        Eigen::MatrixXf A = compute_jacobian_A(x, gyro, bg, dt);
        Eigen::MatrixXf W = compute_jacobian_W(q, dt);
        P_minus = A*P*A.transpose() + W*Q*W.transpose() + Q_bias;
    }

    void correct(const Vec3<float>& acc) 
    {
        Eigen::Quaternionf q(x_hat_minus(0), x_hat_minus(1), x_hat_minus(2), x_hat_minus(3));
        Eigen::MatrixXf H = compute_jacobian_H(q); 
        Eigen::MatrixXf K = P_minus * H.transpose() * (H * P_minus * H.transpose() + R).inverse(); // V is identity

        Vec3<float> z_hat = get_gravity_vector(q);
        // std::cout << "predict Gravity Vector: " << z_hat.transpose() << std::endl;
        // std::cout << "real acc Vector: " << acc.transpose() << std::endl;

        x = x_hat_minus + K * (acc - z_hat);
        P = (Eigen::MatrixXf::Identity(7, 7) - K * H) * P_minus;
        x.head<4>().normalize();
        // x.tail<3>() = z_hat;
        // std::cout << "correct, x: " << x.transpose() << std::endl;
    }

    Eigen::MatrixXf compute_jacobian_A(const Eigen::VectorXf& x, const Vec3<float>& omega, const Vec3<float>& omega_bias, float dt) 
    {
        Eigen::MatrixXf A = Eigen::MatrixXf::Identity(7, 7);
        Vec3<float> omega_correct = omega - omega_bias;

        A.block<4, 4>(0, 0) << 
        1, -dt/2*omega_correct(0), -dt/2*omega_correct(1), -dt/2*omega_correct(2),
        dt/2*omega_correct(0), 1, dt/2*omega_correct(2), -dt/2*omega_correct(1),
        dt/2*omega_correct(1), -dt/2*omega_correct(2), 1, dt/2*omega_correct(0),
        dt/2*omega_correct(2), dt/2*omega_correct(1), -dt/2*omega_correct(0), 1;

        A.block<4, 3>(0, 4) << 
            dt/2*x(1), dt/2*x(2), dt/2*x(3),
            -dt/2*x(0), dt/2*x(3), -dt/2*x(2),
            -dt/2*x(3), -dt/2*x(0), dt/2*x(1),
            dt/2*x(2), -dt/2*x(1), -dt/2*x(0);
        return A;
    }

    Eigen::MatrixXf compute_jacobian_W(const Eigen::Quaternionf& q, float dt) 
    {
        Eigen::MatrixXf W = Eigen::MatrixXf::Zero(7, 3);

        W.block<4, 3>(0, 0) << 
        -0.5f * dt * q.x(), -0.5f * dt * q.y(), -0.5f * dt * q.z(),
        0.5f * dt * q.w(), -0.5f * dt * q.z(),  0.5f * dt * q.y(),
        0.5f * dt * q.z(),  0.5f * dt * q.w(), -0.5f * dt * q.x(),
        -0.5f * dt * q.y(),  0.5f * dt * q.x(),  0.5f * dt * q.w();
        return W;
    }

    Eigen::MatrixXf compute_jacobian_H(const Eigen::Quaternionf& q) {
        Eigen::MatrixXf H = Eigen::MatrixXf::Zero(3, 7);

        H.block<3,4>(0, 0) << -2*q.y(), 2*q.z(), -2*q.w(), 2*q.x(),   
                            2*q.x(), 2*q.w(), 2*q.z(), 2*q.y(),
                            2*q.w(), -2*q.x(), -2*q.y(), 2*q.z();
        return H;
    }

    Vec3<float> get_gravity_vector(const Eigen::Quaternionf& q) 
    {
        return Vec3<float>(
            2*(q.x()*q.z() - q.w()*q.y()) * 9.81f, 
            2*(q.w()*q.x() + q.y()*q.z()) * 9.81f,  
            (q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z()) * (9.81f) 
        );
    }

    Eigen::Matrix4f get_omega_skew(const Vec3<float>& omega) {
        Eigen::Matrix4f skew;
        skew << 0, -omega(0), -omega(1), -omega(2),
                omega(0), 0, omega(2), -omega(1),
                omega(1), -omega(2), 0, omega(0),
                omega(2), omega(1), -omega(0), 0;
        return skew;
    }
};


#endif

// 验证静止状态下姿态收敛
// EKFFilter ekf;
// Vec3<float> acc(0, 0, 9.81f);  // 静止时加速度计读数
// Vec3<float> gyro(0, 0, 0);     // 无旋转
// for (int i = 0; i < 100; i++) {
//     ekf.update(acc, gyro, 0.01f);
//     std::cout << ekf.get_quaternion().coeffs().transpose() << std::endl;
// }