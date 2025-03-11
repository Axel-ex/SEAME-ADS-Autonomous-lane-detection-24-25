#include "../includes/Intersection.hpp"

// Function to calculate y from the parabolic equation
double parabola(double x, double a, double b, double c) {
    return a * x * x + b * x + c;
}

// Function to calculate the left side of the circle equation
// Circle equation is (x - 0.5)^2 + y^2 = R^2
double circleEquation(double x, double a, double b, double c, double R) {
    double y = parabola(x, a, b, c); // Calculate y using the parabolic equation
    return pow(x - 0.5, 2) + pow(y, 2) - R * R;
}

// A simple bisection method to find the root of the equation f(x) = 0
double bisection(double (*f)(double, double, double, double, double), double a, double b, double c, double R, double lower, double upper, double tol = 1e-6) {
    double mid = lower;
    while ((upper - lower) > tol) {
        mid = (lower + upper) / 2;
        if (f(mid, a, b, c, R) == 0.0)
            return mid;
        else if (f(lower, a, b, c, R) * f(mid, a, b, c, R) < 0)
            upper = mid;
        else
            lower = mid;
    }
    return mid;
}

int main() {
    // Constants for the circle and parabola
    double a = 1.0, b = -2.0, c = 1.0; // Example parabola coefficients
    double R = 5.0; // Example radius for the circular path

    // Set an initial range for the bisection method
    double lower = -10.0, upper = 10.0;

    // Find intersection point
    std::cout << "Finding intersection points...\n";
    
    // Using bisection method to find an x for which the equation holds
    double intersectionX = bisection(circleEquation, a, b, c, R, lower, upper);
    double intersectionY = parabola(intersectionX, a, b, c);

    std::cout << "Intersection Point: (" << intersectionX << ", " << intersectionY << ")\n";
    return 0;
}