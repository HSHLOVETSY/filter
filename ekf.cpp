#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <random>
#include "matplotlibcpp.h" // For plotting

using namespace Eigen;
using namespace std;
namespace plt = matplotlibcpp;

// Function to generate random numbers with normal distribution
vector<MatrixXd> generateNoise(int steps, const MatrixXd& covariance) {
    random_device rd;
    mt19937 gen(rd());
    normal_distribution<> dist(0, 1);

    int n = covariance.rows();
    MatrixXd L = covariance.llt().matrixL();

    vector<MatrixXd> noise;
    for (int i = 0; i < steps; ++i) {
        MatrixXd sample(n, 1);
        for (int j = 0; j < n; ++j) {
            sample(j, 0) = dist(gen);
        }
        noise.push_back(L * sample);
    }
    return noise;
}

int main() {
    // Discrete time step
    const double Ts = 0.01;

    // Parameters
    MatrixXd Q(2, 2);   // Process noise covariance
    Q << 0.01, 0, 
         0, 0.01;
    
    MatrixXd R_c(2, 2); // Measurement noise covariance
    R_c << 0.1, 0, 
           0, 0.1;
    
    const double g = 10;  // Gravity acceleration
    const double l = 0.5; // Link length

    // System initialization
    VectorXd x(2);  // System state
    x << M_PI/4, 0;
    
    VectorXd z(2);  // Measurement
    z << 0, 0;
    
    VectorXd x_hat_minus(2); // Prior estimate
    x_hat_minus << 0, 0;
    
    VectorXd x_hat(2); // Posterior estimate
    x_hat << M_PI/4, 0;
    
    MatrixXd P(2, 2); // Posterior estimate error covariance
    P << 1, 0, 
         0, 1;
    
    const int n = x.size();
    const int k_steps = 200;

    // History vectors
    vector<VectorXd> x_history;
    vector<VectorXd> z_history;
    vector<VectorXd> x_hat_minus_history;
    vector<VectorXd> x_hat_history;

    // Store initial values
    x_history.push_back(x);
    z_history.push_back(z);
    x_hat_minus_history.push_back(x_hat_minus);
    x_hat_history.push_back(x_hat);

    // Generate process and measurement noise
    MatrixXd Q_a(2, 2);
    Q_a << 0.01, 0, 
           0, 0.01;
    
    MatrixXd R_a(2, 2);
    R_a << 0.1, 0, 
           0, 0.1;
    
    auto w = generateNoise(k_steps, Q_a); // Process noise
    auto v = generateNoise(k_steps, R_a); // Measurement noise

    // Extended Kalman Filter
    for (int k = 0; k < k_steps; ++k) {
        // System state space equations
        x(0) = x(0) + x(1) * Ts + w[k](0, 0);
        x(1) = x(1) - (g/l) * sin(x(0)) * Ts + w[k](1, 0);

        // Measurement
        z(0) = x(0) + v[k](0, 0);
        z(1) = x(1) + v[k](1, 0);

        // Prior state estimate
        x_hat_minus(0) = x_hat(0) + x_hat(1) * Ts;
        x_hat_minus(1) = x_hat(1) - (g/l) * sin(x_hat(0)) * Ts;

        // Jacobian matrices
        MatrixXd A(2, 2);
        A << 1, Ts, 
             -(g/l)*cos(x_hat(0))*Ts, 1;
        
        MatrixXd W = MatrixXd::Identity(2, 2);
        MatrixXd H_m = MatrixXd::Identity(2, 2);
        MatrixXd V = MatrixXd::Identity(2, 2);

        // Prior estimate error covariance
        MatrixXd P_minus = A * P * A.transpose() + W * Q * W.transpose();

        // Kalman gain
        MatrixXd K = P_minus * H_m.transpose() * 
                    (H_m * P_minus * H_m.transpose() + V * R_c * V.transpose()).inverse();

        // Posterior estimate
        x_hat = x_hat_minus + K * (z - x_hat_minus);

        // Posterior estimate error covariance
        P = (MatrixXd::Identity(n, n) - K * H_m) * P_minus;

        // Save results
        x_history.push_back(x);
        z_history.push_back(z);
        x_hat_minus_history.push_back(x_hat_minus);
        x_hat_history.push_back(x_hat);
    }

    // Calculate mean square error
    double Ems = 0;
    for (int i = 0; i <= k_steps; ++i) {
        double diff = x_hat_history[i](0) - x_history[i](0);
        Ems += diff * diff;
    }
    Ems /= k_steps;

    cout << "Mean Square Error: " << Ems << endl;

    // Prepare data for plotting
    vector<double> time, x_true, z_meas, x_prior, x_posterior;
    for (int i = 0; i <= k_steps; ++i) {
        time.push_back(i);
        x_true.push_back(x_history[i](0));
        z_meas.push_back(z_history[i](0));
        x_prior.push_back(x_hat_minus_history[i](0));
        x_posterior.push_back(x_hat_history[i](0));
    }

    // Plotting
    plt::figure_size(1200, 400);
    plt::plot(time, x_true, "--", {{"label", "True value"}});
    plt::plot(time, z_meas, "*", {{"label", "Measurement"}});
    plt::plot(time, x_prior, "o", {{"label", "Prior estimate"}});
    plt::plot(time, x_posterior, "-", {{"label", "Posterior estimate"}});
    plt::ylim(-3, 3);
    plt::xlabel("Time step");
    plt::ylabel("State value");
    plt::title("Extended Kalman Filter Results");
    plt::grid(true);

    // Add legend - 使用字符串参数指定位置
    plt::legend("best");  // "best"表示自动选择最佳位置
    plt::show();

    return 0;
}