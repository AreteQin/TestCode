#include <iostream>
#include <cmath>
#include <cstdlib>

// Earth's mean radius in meters.
const double R = 6371000.0;
const double PI = 3.14159265358979323846;

int main() {
    // Input GPS coordinates in degrees.
    // Point A: current position (φ1, λ1)
    // Point B: destination (φ2, λ2)
    double phi1_deg = 40.0;     // example current latitude in degrees
    double lambda1_deg = -74.0; // example current longitude in degrees
    double phi2_deg = 41.0;     // example destination latitude in degrees
    double lambda2_deg = -73.0; // example destination longitude in degrees

    // Convert degrees to radians.
    double phi1 = phi1_deg * PI / 180.0;
    double lambda1 = lambda1_deg * PI / 180.0;
    double phi2 = phi2_deg * PI / 180.0;
    double lambda2 = lambda2_deg * PI / 180.0;

    // --- Step 1: Calculate the sides of the spherical triangle NAB ---
    // Let N be the north pole.
    // a = (π/2 - φ2), b = (π/2 - φ1), and c = |λ1 - λ2|
    double a = (PI / 2) - phi2;   // side from North Pole to point B
    double b = (PI / 2) - phi1;   // side from North Pole to point A
    double c_diff = fabs(lambda1 - lambda2); // difference in longitudes

    // --- Step 2: Compute the bearing at point A ---
    // According to the derivation, the bearing (forward azimuth) A is given by:
    // A = arccos( (sin φ2 - sin φ1 * cos(λ1 - λ2)) / (cos φ1 * sin(λ1 - λ2)) )
    //
    // Note: This formula assumes that sin(λ1 - λ2) is non-zero.
    double delta_lambda = lambda1 - lambda2;
    double numerator = sin(phi2) - sin(phi1) * cos(delta_lambda);
    double denominator = cos(phi1) * sin(delta_lambda);

    // Check for potential division by zero.
    if (fabs(denominator) < 1e-10) {
        std::cerr << "Error: Denominator too small, bearing cannot be computed reliably." << std::endl;
        std::exit(EXIT_FAILURE);
    }

    double bearing_rad = acos(numerator / denominator);
    // bearing_rad is in radians. Depending on the sign of sin(Δλ) you might adjust the bearing.
    // Here we use the simple formula from the markdown.

    // --- Step 3: Compute the great-circle distance between A and B ---
    // Using the spherical law of cosines:
    // cos c = sin φ1 sin φ2 + cos φ1 cos φ2 cos(λ1 - λ2)
    double cos_c = sin(phi1) * sin(phi2) + cos(phi1) * cos(phi2) * cos(delta_lambda);
    // Clamp cos_c to the range [-1, 1] to avoid numerical errors.
    if (cos_c > 1.0) cos_c = 1.0;
    if (cos_c < -1.0) cos_c = -1.0;
    double central_angle = acos(cos_c); // c, the central angle between A and B (in radians)
    double distance = R * central_angle; // distance along the surface of the sphere

    // --- Output the results ---
    std::cout << "Bearing (radians): " << bearing_rad << std::endl;
    std::cout << "Bearing (degrees): " << bearing_rad * 180.0 / PI << std::endl;
    std::cout << "Distance (meters): " << distance << std::endl;

    return 0;
}
