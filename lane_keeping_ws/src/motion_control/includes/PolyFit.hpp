#ifndef POLY_HPP
#define POLY_HPP

#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <cmath>
#include <iomanip>
#include <algorithm>
#include "Quartic.hpp"

using namespace std;

#define MAXIT 100
#define EPS 3.0e-7
#define FPMIN 1.0e-30

#define STOP 1.0e-8
#define TINY 1.0e-30

double  calculatePoly(const double x, const double* a, const size_t n);
std::vector<double> calculate(double *x, double *y, size_t degree, size_t n);
std::vector<double> find_best_fit(double *x, double *y, size_t n);


#endif