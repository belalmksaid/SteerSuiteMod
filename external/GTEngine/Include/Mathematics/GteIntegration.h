// David Eberly, Geometric Tools, Redmond WA 98052
// Copyright (c) 1998-2016
// Distributed under the Boost Software License, Version 1.0.
// http://www.boost.org/LICENSE_1_0.txt
// http://www.geometrictools.com/License/Boost/LICENSE_1_0.txt
// File Version: 2.1.0 (2016/01/25)

#pragma once

#include <Mathematics/GteRootsPolynomial.h>
#include <array>
#include <functional>
#include <vector>

namespace gte
{

template <typename Real>
class Integration
{
public:
    // A simple algorithm, but slow to converge as the number of samples is
    // increased.  The 'numSamples' needs to be two or larger.
    static Real TrapezoidRule(int numSamples, Real a, Real b,
        std::function<Real(Real)> const& integrand);

    // The trapezoid rule is used to generate initial estimates, but then
    // Richardson extrapolation is used to improve the estimates.  This is
    // preferred over TrapezoidRule.  The 'order' must be positive.
    static Real Romberg(int order, Real a, Real b,
        std::function<Real(Real)> const& integrand);

    // Gaussian quadrature estimates the integral of a function f(x) defined
    // on [-1,1] using
    //   integral_{-1}^{1} f(t) dt = sum_{i=0}^{n-1} c[i]*f(r[i])
    // where r[i] are the roots to the Legendre polynomial p(t) of degree n
    // and c[i] = integral_{-1}^{1} prod_{j=0,j!=i} (t-r[j]/(r[i]-r[j]) dt.
    // To integrate over [a,b], a transformation to [-1,1] is applied
    // internally:  x - ((b-a)*t + (b+a))/2.  The Legendre polynomials are
    // generated by
    //   P[0](x) = 1, P[1](x) = x,
    //   P[k](x) = ((2*k-1)*x*P[k-1](x) - (k-1)*P[k-2](x))/k, k >= 2
    // Implementing the polynomial generation is simple, and computing the
    // roots requires a numerical method for finding polynomial roots.  The
    // challenging task is to develop an efficient algorithm for computing
    // the coefficients c[i] for a specified degree.  The 'degree' must be
    // two or larger.

    static void ComputeQuadratureInfo(int degree, std::vector<Real>& roots,
        std::vector<Real>& coefficients);

    static Real GaussianQuadrature(std::vector<Real> const& roots,
        std::vector<Real>const & coefficients, Real a, Real b,
        std::function<Real(Real)> const& integrand);
};


template <typename Real>
Real Integration<Real>::TrapezoidRule(int numSamples, Real a, Real b,
    std::function<Real(Real)> const& integrand)
{
    Real h = (b - a) / (Real)(numSamples - 1);
    Real result = ((Real)0.5) * (integrand(a) + integrand(b));
    for (int i = 1; i <= numSamples - 2; ++i)
    {
        result += integrand(a + i*h);
    }
    result *= h;
    return result;
}

template <typename Real>
Real Integration<Real>::Romberg(int order, Real a, Real b,
    std::function<Real(Real)> const& integrand)
{
    Real const half = (Real)0.5;
    std::vector<std::array<Real, 2>> rom(order);
    Real h = b - a;
    rom[0][0] = half * h * (integrand(a) + integrand(b));
    for (int i0 = 2, p0 = 1; i0 <= order; ++i0, p0 *= 2, h *= half)
    {
        // Approximations via the trapezoid rule.
        Real sum = (Real)0;
        int i1;
        for (i1 = 1; i1 <= p0; ++i1)
        {
            sum += integrand(a + h * (i1 - half));
        }

        // Richardson extrapolation.
        rom[0][1] = half * (rom[0][0] + h * sum);
        for (int i2 = 1, p2 = 4; i2 < i0; ++i2, p2 *= 4)
        {
            rom[i2][1] = (p2*rom[i2 - 1][1] - rom[i2 - 1][0]) / (p2 - 1);
        }

        for (i1 = 0; i1 < i0; ++i1)
        {
            rom[i1][0] = rom[i1][1];
        }
    }

    Real result = rom[order - 1][0];
    return result;
}

template <typename Real>
void Integration<Real>::ComputeQuadratureInfo(int degree,
    std::vector<Real>& roots, std::vector<Real>& coefficients)
{
    Real const zero = (Real)0;
    Real const one = (Real)1;
    Real const half = (Real)0.5;

    std::vector<std::vector<Real>> poly(degree + 1);

    poly[0].resize(1);
    poly[0][0] = one;

    poly[1].resize(2);
    poly[1][0] = zero;
    poly[1][1] = one;

    for (int n = 2; n <= degree; ++n)
    {
        Real mult0 = (Real)(n - 1) / (Real)n;
        Real mult1 = (Real)(2 * n - 1) / (Real)n;

        poly[n].resize(n + 1);
        poly[n][0] = -mult0 * poly[n - 2][0];
        for (int i = 1; i <= n - 2; ++i)
        {
            poly[n][i] = mult1 * poly[n - 1][i - 1] - mult0 * poly[n - 2][i];
        }
        poly[n][n - 1] = mult1 * poly[n - 1][n - 2];
        poly[n][n] = mult1 * poly[n - 1][n - 1];
    }

    roots.resize(degree);
    RootsPolynomial<Real>::Find(degree, &poly[degree][0], 2048, &roots[0]);

    coefficients.resize(roots.size());
    size_t n = roots.size() - 1;
    std::vector<Real> subroots(n);
    for (size_t i = 0; i < roots.size(); ++i)
    {
        Real denominator = (Real)1;
        for (size_t j = 0, k = 0; j < roots.size(); ++j)
        {
            if (j != i)
            {
                subroots[k++] = roots[j];
                denominator *= roots[i] - roots[j];
            }
        }

        std::array<Real, 2> delta =
        {
            -one - subroots.back(),
            +one - subroots.back()
        };

        std::vector<std::array<Real, 2>> weights(n);
        weights[0][0] = half * delta[0] * delta[0];
        weights[0][1] = half * delta[1] * delta[1];
        for (size_t k = 1; k < n; ++k)
        {
            Real dk = (Real)k;
            Real mult = -dk / (dk + (Real)2);
            weights[k][0] = mult * delta[0] * weights[k - 1][0];
            weights[k][1] = mult * delta[1] * weights[k - 1][1];
        }

        struct Info
        {
            int numBits;
            std::array<Real, 2> product;
        };

        std::vector<Info> info(1 << (n - 1));
        info[0].numBits = 0;
        info[0].product[0] = one;
        info[0].product[1] = one;
        for (int ipow = 1, r = 0; ipow < (1 << (n - 1)); ipow <<= 1, ++r)
        {
            info[ipow].numBits = 1;
            info[ipow].product[0] = -one - subroots[r];
            info[ipow].product[1] = +one - subroots[r];
            for (int m = 1, j = ipow + 1; m < ipow; ++m, ++j)
            {
                info[j].numBits = info[m].numBits + 1;
                info[j].product[0] =
                    info[ipow].product[0] * info[m].product[0];
                info[j].product[1] =
                    info[ipow].product[1] * info[m].product[1];
            }
        }

        std::vector<std::array<Real, 2>> sum(n);
        std::array<Real, 2> zero2 = { zero, zero };
        std::fill(sum.begin(), sum.end(), zero2);
        for (size_t k = 0; k < info.size(); ++k)
        {
            sum[info[k].numBits][0] += info[k].product[0];
            sum[info[k].numBits][1] += info[k].product[1];
        }

        std::array<Real, 2> total = zero2;
        for (size_t k = 0; k < n; ++k)
        {
            total[0] += weights[n - 1 - k][0] * sum[k][0];
            total[1] += weights[n - 1 - k][1] * sum[k][1];
        }

        coefficients[i] = (total[1] - total[0]) / denominator;
    }
}

template <typename Real>
Real Integration<Real>::GaussianQuadrature(std::vector<Real> const& roots,
    std::vector<Real>const & coefficients, Real a, Real b,
    std::function<Real(Real)> const& integrand)
{
    Real const half = (Real)0.5;
    Real radius = half * (b - a);
    Real center = half * (b + a);
    Real result = (Real)0;
    for (size_t i = 0; i < roots.size(); ++i)
    {
        result += coefficients[i] * integrand(radius*roots[i] + center);
    }
    result *= radius;
    return result;
}


}
