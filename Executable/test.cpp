// Implementation 1
#include <iostream>
#include <cmath>
#include <cstdlib>

const double PI = 3.14159265358979323846;
const double R = 6371.0; // Earth's radius in kilometers

int main()
{
    // Input GPS coordinates in degrees:
    // Point A: current position (φ₁, λ₁)
    // Point B: destination (φ₂, λ₂)
    double phi1_deg = 45; // φ₁: current latitude in degrees
    double lambda1_deg = -73; // λ₁: current longitude in degrees
    double phi2_deg = 45; // φ₂: destination latitude in degrees
    double lambda2_deg = -72; // λ₂: destination longitude in degrees

    // Convert degrees to radians.
    double phi1 = phi1_deg * PI / 180.0;
    double lambda1 = lambda1_deg * PI / 180.0;
    double phi2 = phi2_deg * PI / 180.0;
    double lambda2 = lambda2_deg * PI / 180.0;

    // --- Step 1: Calculate the sides of the spherical triangle NAB ---
    // Let N be the north pole.
    // a = (π/2 - φ₂), b = (π/2 - φ₁), and C = λ₁ - λ₂ (angle at the north pole)
    double a = (PI / 2.0) - phi2; // side from North Pole to point B
    double b = (PI / 2.0) - phi1; // side from North Pole to point A
    double C = lambda2 - lambda1; // angle at the north pole

    // --- Step 2: Calculate the bearing A using the derived formula ---
    // According to the derivation:
    // tan A = (sin a * sin b * sin(λ₁ - λ₂)) / (cos a - cos b * (cos a * cos b + sin a * sin b * cos C))
    double numerator = sin(a) * sin(b) * sin(C);
    double denominator = cos(a) - cos(b) * (cos(a) * cos(b) + sin(a) * sin(b) * cos(C));

    // Check for division by zero.
    if (fabs(denominator) < 1e-10)
    {
        std::cerr << "Error: Denominator too small, bearing cannot be computed reliably." << std::endl;
        std::exit(EXIT_FAILURE);
    }
    double A_rad = atan2(numerator, denominator);

    // Optionally, convert the bearing to degrees.
    double A_deg = A_rad * 180.0 / PI;

    std::cout << "Bearing (radians): " << A_rad << std::endl;
    std::cout << "Bearing (degrees): " << A_deg << std::endl;

    // --- Step 3: Compute the great-circle distance between A and B ---
    // Using the spherical law of cosines:
    // cos c = sin φ1 sin φ2 + cos φ1 cos φ2 cos(λ1 - λ2)
    double cos_c = sin(phi1) * sin(phi2) + cos(phi1) * cos(phi2) * cos(C);
    // Clamp cos_c to the range [-1, 1] to avoid numerical errors.
    if (cos_c > 1.0) cos_c = 1.0;
    if (cos_c < -1.0) cos_c = -1.0;
    double central_angle = acos(cos_c); // c, the central angle between A and B (in radians)
    double distance = R * central_angle; // distance along the surface of the sphere
    std::cout << "Distance (km): " << distance << std::endl;

    return 0;
}
