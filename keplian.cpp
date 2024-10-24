#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>

// Gravitational constant (Earth)
const double mu = 398600.4418;  // in km^3/s^2

// Helper function to convert degrees to radians
double degrees_to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Solve Kepler's equation for Eccentric Anomaly using iterative method
double solve_kepler(double mean_anomaly, double eccentricity) {
    double E = mean_anomaly;  // Start with an approximation
    for (int i = 0; i < 100; ++i) {
        double delta = E - eccentricity * sin(E) - mean_anomaly;
        if (fabs(delta) < 1e-6) break;  // Converged
        E -= delta / (1 - eccentricity * cos(E));
    }
    return E;
}
