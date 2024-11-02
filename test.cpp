#include <matplot/matplot.h>
#include <cmath>
#include <vector>

int main() {
    // Set up a range for omega
    std::vector<double> omega;
    for (double i = 0.01; i <= 100; i += 0.01) {
        omega.push_back(i);
    }

    // Compute M values
    std::vector<double> M;
    for (auto w : omega) {
        double term1 = w / (1 + w * w);
        double term2 = 1 / (1 + w * w);
        double magnitude = 20 * std::log10(std::sqrt(term1 * term1 + term2 * term2));
        M.push_back(magnitude);
    }

    // Plot M versus omega with logarithmic scale on x-axis
    matplot::semilogx(omega, M);
    matplot::title("Plot of M = 20log(sqrt((w/(1+w^2))^2 + (1/(1+w^2))^2))");
    matplot::xlabel("omega (log scale)");
    matplot::ylabel("M (dB)");
    matplot::show();

    std::vector<double> omega_ = matplot::logspace(-2, 2, 100);  // 0.1 to 100 with 100 points
    std::vector<double> phi;

    // Compute phi values
    for (double w : omega_) {
        phi.push_back(std::atan(-w));
    }

    // Create the plot
    matplot::semilogx(omega_, phi);
    matplot::xlabel("ω");
    matplot::ylabel("ϕ (radians)");
    matplot::title("Plot of ϕ = arctan(-ω)");

    // Show the plot
    matplot::show();

    return 0;
}
