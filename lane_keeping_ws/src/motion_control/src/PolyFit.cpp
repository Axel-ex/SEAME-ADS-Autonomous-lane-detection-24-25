#include "../includes/PolyFit.hpp"

/*
 * zlib License
 *
 * Regularized Incomplete Beta Function
 *
 * Copyright (c) 2016, 2017 Lewis Van Winkle
 * http://CodePlea.com
 *
 * This software is provided 'as-is', without any express or implied
 * warranty. In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgement in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

// Adapted from https://github.com/codeplea/incbeta
double incbeta(double a, double b, double x)
{
    if (x < 0.0 || x > 1.0)
        return 1.0 / 0.0;

    if (a <= 0.)
    {
        std::cout << "Warning: a should be >0";
        return 0.;
    }

    if (b <= 0.)
    {
        std::cout << "Warning: b should be >0";
        return 0.;
    }
    /*The continued fraction converges nicely for x < (a+1)/(a+b+2)*/
    if (x > (a + 1.0) / (a + b + 2.0))
        return (
            1.0 -
            incbeta(b, a, 1.0 - x)); /*Use the fact that beta is symmetrical.*/

    /*Find the first part before the continued fraction.*/
    const double lbeta_ab = lgamma(a) + lgamma(b) - lgamma(a + b);
    const double front = exp(log(x) * a + log(1.0 - x) * b - lbeta_ab) / a;

    /*Use Lentz's algorithm to evaluate the continued fraction.*/
    double f = 1.0, c = 1.0, d = 0.0;

    int i, m;
    for (i = 0; i <= 200; ++i)
    {
        m = i / 2;

        double numerator;
        if (i == 0)
            numerator = 1.0; /*First numerator is 1.0.*/
        else if (i % 2 == 0)
            numerator = (m * (b - m) * x) /
                        ((a + 2.0 * m - 1.0) * (a + 2.0 * m)); /*Even term.*/
        else
            numerator = -((a + m) * (a + b + m) * x) /
                        ((a + 2.0 * m) * (a + 2.0 * m + 1)); /*Odd term.*/
        /*Do an iteration of Lentz's algorithm.*/
        d = 1.0 + numerator * d;
        if (fabs(d) < TINY)
            d = TINY;
        d = 1.0 / d;

        c = 1.0 + numerator / c;
        if (fabs(c) < TINY)
            c = TINY;

        const double cd = c * d;
        f *= cd;

        /*Check for stop.*/
        if (fabs(1.0 - cd) < STOP)
            return front * (f - 1.0);
    }
    return 1.0 / 0.0; /*Needed more loops, did not converge.*/
}

double invincbeta(double y, double alpha, double beta)
{

    if (y <= 0.)
        return 0.;
    else if (y >= 1.)
        return 1.;
    if (alpha <= 0.)
    {
        std::cout << "Warning: alpha should be >0";
        return 0.;
    }
    if (beta <= 0.)
    {
        std::cout << "Warning: beta should be >0";
        return 0.;
    }

    double x = 0.5, a = 0, b = 1, precision = 1.e-8, binit = y;
    double bcur = incbeta(alpha, beta, x);

    while (fabs(bcur - binit) > precision)
    {

        if ((bcur - binit) < 0)
            a = x;
        else
            b = x;
        x = (a + b) * 0.5;
        bcur = incbeta(alpha, beta, x);
    }
    return x;
}

// Initialize a 2D array
// **************************************************************
double** Make2DArray(const size_t rows, const size_t cols)
{

    double** array = new double*[rows];
    for (size_t i = 0; i < rows; i++)
        array[i] = new double[cols];
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            array[i][j] = 0.;
    return array;
}

// Free a 2D array
// **************************************************************
void Free2DArray(double** array, const size_t rows)
{
    for (size_t i = 0; i < rows; i++)
        delete[] array[i];
    delete[] array;
}

// Transpose a 2D array
// **************************************************************
double** MatTrans(double** array, const size_t rows, const size_t cols)
{
    double** arrayT = Make2DArray(cols, rows);
    for (size_t i = 0; i < rows; i++)
        for (size_t j = 0; j < cols; j++)
            arrayT[j][i] = array[i][j];
    return arrayT;
}

// Perform the multiplication of matrix A[m1,m2] by B[m2,m3]
// **************************************************************
double** MatMul(const size_t m1, const size_t m2, const size_t m3, double** A,
                double** B)
{
    double** array = Make2DArray(m1, m3);
    for (size_t i = 0; i < m1; i++)
    {
        for (size_t j = 0; j < m3; j++)
        {
            array[i][j] = 0.;
            for (size_t m = 0; m < m2; m++)
                array[i][j] += A[i][m] * B[m][j];
        }
    }
    return array;
}

// Perform the multiplication of matrix A[m1,m2] by vector v[m2,1]
// **************************************************************
void MatVectMul(const size_t m1, const size_t m2, double** A, double* v,
                double* Av)
{
    for (size_t i = 0; i < m1; i++)
    {
        Av[i] = 0.;
        for (size_t j = 0; j < m2; j++)
            Av[i] += A[i][j] * v[j];
    }
}

double determinant(double** a, const size_t n)
{
    double det = 1.0;
    for (size_t i = 0; i < n; i++)
    {
        size_t pivot = i;
        for (size_t j = i + 1; j < n; j++)
        {
            if (abs(a[j][i]) > abs(a[pivot][i]))
                pivot = j;
        }
        if (pivot != i)
        {
            swap(a[i], a[pivot]);
            det *= -1;
        }
        if (a[i][i] == 0)
            return 0;
        det *= a[i][i];
        for (size_t j = i + 1; j < n; j++)
        {
            double factor = a[j][i] / a[i][i];
            for (size_t k = i + 1; k < n; k++)
                a[j][k] -= factor * a[i][k];
        }
    }
    return det;
}

// Perform the
// **************************************************************
void transpose(double** num, double** fac, double** inverse, const size_t r)
{

    double** b = Make2DArray(r, r);
    double deter;

    for (size_t i = 0; i < r; i++)
        for (size_t j = 0; j < r; j++)
            b[i][j] = fac[j][i];

    deter = determinant(num, r);

    for (size_t i = 0; i < r; i++)
        for (size_t j = 0; j < r; j++)
            inverse[i][j] = b[i][j] / deter;
    Free2DArray(b, r);
}

// Calculates the cofactors
// **************************************************************
void cofactor(double** num, double** inverse, const size_t f)
{

    double** b = Make2DArray(f, f);
    double** fac = Make2DArray(f, f);

    size_t m;
    size_t n;

    for (size_t q = 0; q < f; q++)
    {
        for (size_t p = 0; p < f; p++)
        {
            m = 0;
            n = 0;
            for (size_t i = 0; i < f; i++)
            {
                for (size_t j = 0; j < f; j++)
                {
                    if (i != q && j != p)
                    {
                        b[m][n] = num[i][j];
                        if (n < (f - 2))
                            n++;
                        else
                        {
                            n = 0;
                            m++;
                        }
                    }
                }
            }
            fac[q][p] = pow(-1, q + p) * determinant(b, f - 1);
        }
    }

    transpose(num, fac, inverse, f);
    Free2DArray(b, f);
    Free2DArray(fac, f);
}

// Perform the fit of data n data points (x,y) with a polynomial of order k
// **************************************************************
void PolyFit(const double* x, double* y, const size_t n, const size_t k,
             const bool fixedinter, const double fixedinterval,
             double* coeficients, double** Weights, double** XTWXInv)
{

    // Definition of variables
    // **************************************************************
    double** X = Make2DArray(n, k + 1); // [n,k+1]
    double** XT;                        // [k+1,n]
    double** XTW;                       // [k+1,n]
    double** XTWX;                      // [k+1,k+1]

    double* XTWY = new double[k + 1];
    double* Y = new double[n];

    size_t begin = 0;
    if (fixedinter)
        begin = 1;

    // Initialize X
    // **************************************************************
    for (size_t i = 0; i < n; i++)
    {
        for (size_t j = begin; j < (k + 1); j++)
            X[i][j] = pow(x[i], j);
    }

    // Matrix calculations
    // **************************************************************
    XT = MatTrans(X, n, k + 1);             // Calculate XT
    XTW = MatMul(k + 1, n, n, XT, Weights); // Calculate XT*W
    XTWX = MatMul(k + 1, n, k + 1, XTW, X); // Calculate (XTW)*X

    if (fixedinter)
        XTWX[0][0] = 1.;

    cofactor(XTWX, XTWXInv, k + 1); // Calculate (XTWX)^-1

    for (size_t m = 0; m < n; m++)
    {
        if (fixedinter)
            Y[m] = y[m] - fixedinterval;
        else
            Y[m] = y[m];
    }
    MatVectMul(k + 1, n, XTW, Y, XTWY); // Calculate (XTW)*Y
    MatVectMul(k + 1, k + 1, XTWXInv, XTWY,
               coeficients); // Calculate coeficients = (XTWXInv)*XTWY

    if (fixedinter)
        coeficients[0] = fixedinterval;

    Free2DArray(X, n);
    delete[] XTWY;
    delete[] Y;
    Free2DArray(XT, k + 1);
    Free2DArray(XTW, k + 1);
    Free2DArray(XTWX, k + 1);
}

// Calculate the polynomial at a given x value
// **************************************************************
double calculatePoly(const double x, const double* a, const size_t n)
{
    double poly = 0.;

    for (size_t i = 0; i < n + 1; i++)
        poly += a[i] * pow(x, i);
    return poly;
}

// Calculate the weights matrix
// **************************************************************
void CalculateWeights(const double* erry, double** Weights, const size_t n,
                      const int type)
{
    for (size_t i = 0; i < n; i++)
    {
        switch (type)
        {
        case 0:
            Weights[i][i] = 1.;
            break;
        case 1:
            Weights[i][i] = erry[i];
            break;
        case 2:
            if (erry[i] > 0.)
                Weights[i][i] = 1. / (erry[i] * erry[i]);
            else
                Weights[i][i] = 0.;
            break;
        }
    }
}

// Display the coefficients of the polynomial
// **************************************************************
void DisplayCoefs(const size_t k, const double* coefbeta)
{
    cout << "Polynomial coefficients" << endl;
    cout << "Coeff\tValue" << endl;

    for (size_t i = 0; i < (k + 1); i++)
    {
        cout << "A" << i << "\t";
        cout << coefbeta[i] << "\t";
        cout << endl;
    }
}

std::vector<double> calculate(double* x, double* y, size_t degree, size_t n)
{
    bool fixedinter = false; // Fixed the intercept (coefficient A0)
    int wtype = 0; // Weight: 0 = none (default), 1 = sigma, 2 = 1/sigma^2
    double fixedinterval = 0.; // The fixed intercept value (if applicable)
    double erry[] = {};        // Data points (err on y) (if applicable)

    // Definition of other variables
    // **************************************************************
    size_t nstar = 0; // equal to n (fixed intercept) or (n-1) not fixed
    double* coefbeta = new double[degree + 1]; // Coefficients of the polynomial

    double** XTWXInv; // Matrix XTWX Inverse [degree+1,degree+1]
    double** Weights; // Matrix Weights [n,n]

    // Initialize values
    // **************************************************************
    cout << x << endl;
    nstar = n - 1;
    if (fixedinter)
        nstar = n;

    if (fixedinter)
        cout << "A0 is fixed!" << endl;

    if (degree > nstar)
    {
        cout << "The polynomial order is too high. Max should be " << n
             << " for adjustable A0 ";
        cout << "and " << n - 1 << " for fixed A0. ";
        cout << "Program stopped" << endl;
        exit(1);
    }

    if (degree == nstar)
    {
        cout << "The degree of freedom is equal to the number of points. ";
        cout << "The fit will be exact." << endl;
    }

    XTWXInv = Make2DArray(degree + 1, degree + 1);
    Weights = Make2DArray(n, n);

    // Build the weight matrix
    // **************************************************************
    CalculateWeights(erry, Weights, n, wtype);

    if (determinant(Weights, n) == 0.)
    {
        cout << "One or more points have 0 error. Review the errors on points "
                "or use no weighting. ";
        cout << "Program stopped" << endl;
        exit(1);
    }

    // Calculate the coefficients of the fit
    // **************************************************************
    PolyFit(x, y, n, degree, fixedinter, fixedinterval, coefbeta, Weights,
            XTWXInv);
    // DisplayCoefs(degree, coefbeta);

    Free2DArray(XTWXInv, degree + 1);
    Free2DArray(Weights, n);
    std::vector<double> vec_coef(coefbeta, coefbeta + degree + 1);
    delete[] coefbeta;
    return vec_coef;
}

double solveQuadratic(double a, double b, double c)
{
    double t_plus = (-1 * b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double t_minu = (-1 * b - sqrt(b * b - 4 * a * c)) / (2 * a);
    if (t_plus >= 0 && (t_minu <= 0 || t_plus <= t_minu))
        return (t_plus);
    else if (t_minu >= 0)
        return (t_minu);
    else
        return (-1);
}
