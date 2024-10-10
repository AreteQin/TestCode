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

double EKF(double x, double y, double u, double Q, double R, double &P) {
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
    // posterior covariance
    P = P - K * dh * P;
    return est_posterior;
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
        dh = H_jacobian(est_posterior);
        K = P * dh / (dh * P * dh + R);
        est_posterior = est_prior + K * (y - h_measure(est_posterior)-dh*(est_prior-est_posterior));
    }
    // posterior covariance
    P = P - K * dh * P;
    // P = (1 - K * dh) * P * (1 - K * dh) + K * R * K;
    return est_posterior;
}

double UKF(double x, double y, double u, double Q, double R, double &P, double k) {
    // predict
    double L = sqrt(P);
    std::vector<double> sigma_points;
    for (int i = 0; i < 3; i++) {
        if (i == 0) {
            sigma_points.push_back(x);
        } else {
            sigma_points.push_back(x + sqrt(1 + k) * L);
            sigma_points.push_back(x - sqrt(1 + k) * L);
        }
    }
    std::vector<double> est_prior, alphas, y_est, P_y, P_x, P_xy;
    double est_prior_sum = 0;
    for (int i = 0; i < sigma_points.size(); i++) {
        // prior estimate
        est_prior.push_back(f_process(sigma_points[i], u));
        // alpha, The sum of all alphas is 1

        if (i == 0) {
            alphas.push_back(k / (1 + k));
        } else {
            alphas.push_back(0.5 * (k / (1 + k)));
        }
        est_prior_sum += alphas[i] * est_prior[i];
    }
    // prior covariance
    double P_x_sum = 0;
    for (int i = 0; i < sigma_points.size(); i++) {
        // prior covariance
        P_x.push_back(alphas[i] * (est_prior[i] - est_prior_sum) * (est_prior[i] - est_prior_sum) + Q);
        P_x_sum += P_x[i];
    }
    // correction
    double y_est_sum = 0;
    for (int i = 0; i < sigma_points.size(); i++) {
        y_est.push_back(alphas[i] * h_measure(est_prior[i]));
        y_est_sum += y_est[i];
    }
    double P_y_sum = 0;
    for (int i = 0; i < sigma_points.size(); i++) {
        P_y.push_back(alphas[i] * (y_est[i] - y_est_sum) * (y_est[i] - y_est_sum) + R);
        P_y_sum += P_y[i];
    }
    double P_xy_sum = 0;
    for (int i = 0; i < sigma_points.size(); i++) {
        P_xy.push_back(alphas[i] * (est_prior[i] - est_prior_sum) * (y_est[i] - y_est_sum));
        P_xy_sum += P_xy[i];
    }
    // kalman gain
    double K = P_xy_sum / P_y_sum;
    // posterior estimate
    double est_posterior = est_prior_sum + K * (y - y_est_sum);
    // posterior covariance
    P = P_x_sum - K * P_y_sum * K;
    return est_posterior;
}

int main() {
    int total_simulation_times = 50;
    int iteration_times = 10;
    double Q = 1; // state noise covariance
    double R = 1; // measurement noise covariance
    double P = Q;
    double P_IEKF = P;
    double P_EKF = P;
    double P_UKF = P;
    // generate a random value between 0 and 1 for initial state x
    cv::RNG rng; // OpenCV random number generator
    double x_init = rng.uniform(0.1, 1.0);
    std::vector<double> x_real, est_IEKF, est_EKF, est_UKF, time_stamp, IEKF_errors, EKF_errors, UKF_errors;
    double IEKF_error_sum = 0, EKF_error_sum = 0, UKF_error_sum = 0;
    time_stamp.push_back(0);
    IEKF_errors.push_back(0);
    EKF_errors.push_back(0);
    UKF_errors.push_back(0);
    x_real.push_back(x_init);
    est_IEKF.push_back(x_init);
    est_EKF.push_back(x_init);
    est_UKF.push_back(x_init);
    for (int i = 1; i < total_simulation_times; i++) {
        time_stamp.push_back(i);
        double x_real_temp = f_process(x_real[i - 1], i - 1) + rng.gaussian(Q); // i is the control input
        x_real.push_back(x_real_temp);
        double y = h_measure(x_real_temp) + rng.gaussian(R);
        est_EKF.push_back(EKF(est_EKF[i - 1], y, i - 1, Q, R, P_EKF));
        est_IEKF.push_back(IEKF(est_IEKF[i - 1], y, i - 1, Q, R, P_IEKF, iteration_times));
        est_UKF.push_back(UKF(est_UKF[i - 1], y, i - 1, Q, R, P_UKF, 2));
        // calculate the errors
        IEKF_errors.push_back(x_real[i] - est_IEKF[i]);
        EKF_errors.push_back(x_real[i] - est_EKF[i]);
        UKF_errors.push_back(x_real[i] - est_UKF[i]);
        IEKF_error_sum += abs(IEKF_errors[i]);
        EKF_error_sum += abs(EKF_errors[i]);
        UKF_error_sum += abs(UKF_errors[i]);
    }
    std::cout<< "IEKF error sum: " << IEKF_error_sum << std::endl;
    std::cout<< "EKF error sum: " << EKF_error_sum << std::endl;
    std::cout<< "UKF error sum: " << UKF_error_sum << std::endl;
    // plot the real x and estimated x
    // Plot given data and real plot
    plt::plot(time_stamp, x_real);
    plt::plot(time_stamp, est_EKF); // orange line
//    plt::plot(time_stamp, EKF_errors); // green line
    plt::title("EKF figure");
    plt::show();
    plt::plot(time_stamp, x_real);
    plt::plot(time_stamp, est_IEKF);
//    plt::plot(time_stamp, IEKF_errors);
    plt::title("iEKF figure");
    plt::show();
    plt::plot(time_stamp, x_real);
    plt::plot(time_stamp, est_UKF);
//    plt::plot(time_stamp, UKF_errors);
    plt::title("UKF figure");
    plt::show();
    return 0;
}