#ifndef POLY_HPP
#define POLY_HPP

#include <cmath>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include "Quartic.hpp"

#include <iostream>
#include <math.h>
#include <vector>
using namespace std;

#define MAXIT 100
#define FPMIN 1.0e-30
constexpr double POLY_EPS = 3.0e-7;

#define STOP 1.0e-8
#define TINY 1.0e-30

double  calculatePoly(const double x, const double* a, const size_t n);
std::vector<double> find_best_fit(double *x, double *y, size_t n);

double solveQuadratic(double a, double b, double c, bool is_right_lane);

#endif
