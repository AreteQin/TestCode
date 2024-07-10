#include <opencv2/core/core.hpp>
#include <vector>
#include <matplotlibcpp.h>

namespace plt = matplotlibcpp;

double f_process(double x, double u) {
    return 5 * cos(x) * cos(u);
}

double h_measure(double x) {
    return x * x / 2 + 10;
}

double F_jacobian(double x, double u) {
    return 5 * cos(u) * (-sin(x));
}

double H_jacobian(double x) {
    return x;
}

double IEKF(double x, double y, double u, double Q, double R, double &P, int iteration_times) {
    // predict
    // prior estimate
    double est_prior = f_process(x, u);
    double df = F_jacobian(est_prior, u);
    double dh = H_jacobian(est_prior);
    // prior covariance
    P = df * P * df + Q;
    // update
    // kalman gain
    double K = P * dh / (dh * P * dh + R);
    // posterior estimate
    double est_posterior = est_prior + K * (y - h_measure(est_prior));
    for (int j = 0; j < iteration_times; j++) {
//            df = 0.5 * est_IEKF[i] + 25 * (est_IEKF[i] / (1 + est_IEKF[i] * est_IEKF[i]));
        dh = H_jacobian(est_posterior);
//            P = df * P * df + Q;
        K = P * dh / (dh * P * dh + R);
        est_posterior = est_posterior + K * (y - h_measure(est_posterior));
//            P = P - K * dh * P;
    }
    // posterior covariance
    P = P - K * dh * P;
    return est_posterior;
}

int main() {
    int total_simulation_times = 50;
    int iteration_times = 10;
    double Q = 1; // state noise covariance
    double R = 1; // measurement noise covariance
    double P = Q;
    double P_IEKF = P;
    // generate a random value between 0 and 1 for initial state x
    cv::RNG rng; // OpenCV random number generator
    double x_init = rng.uniform(0.1, 1.0);
    std::vector<double> x_real, est_IEKF, time_stamp, IEKF_errors;
    double IEKF_error_sum = 0, EKF_error_sum = 0, UKF_error_sum = 0;
    time_stamp.push_back(0);
    IEKF_errors.push_back(0);
    x_real.push_back(x_init);
    est_IEKF.push_back(x_init);
    for (int i = 1; i < total_simulation_times; i++) {
        time_stamp.push_back(i);
        double x_real_temp = f_process(x_real[i - 1], i - 1) + rng.gaussian(Q); // i is the control input
        x_real.push_back(x_real_temp);
        double y = h_measure(x_real_temp) + rng.gaussian(R);
        est_IEKF.push_back(IEKF(est_IEKF[i - 1], y, i - 1, Q, R, P_IEKF, iteration_times));
        // calculate the errors
        IEKF_errors.push_back(x_real[i] - est_IEKF[i]);
        IEKF_error_sum += abs(IEKF_errors[i]);
    }
    std::cout<< "IEKF error sum: " << IEKF_error_sum << std::endl;
    // plot the real x and estimated x
    // Plot given data and real plot
    plt::plot(time_stamp, x_real);
    plt::plot(time_stamp, est_IEKF);
    plt::title("iEKF figure");
    plt::show();
    return 0;
}