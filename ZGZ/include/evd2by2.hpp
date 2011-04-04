/*
 * 2by2evd.hpp
 *
 *  Created on: Apr 2, 2011
 *      Author: yasir
 */

#ifndef EVD2by2_HPP_
#define EVD2by2_HPP_

#include <math.h>
#include <Eigen/Core>

using namespace Eigen;
#define tolerance 0.1e-20

#define square(x) x*x

void solve_eig(Matrix2f& P,Vector2d& lambda, Matrix2d& v, double& theeta)
    //double* lambda1, double *v1x, double*v1y,
    //double* lambda2, double *v2x, double*v2y )
{
	double A = P(0,0);
	double B = P(0,1);
	double C = P(1,0);
	double D = P(1,1);

    if(B*C <= tolerance ) {
        lambda(0) = A;
        lambda(1) = D;
        v = Matrix2d::Identity();
        theeta = 0;
        return;
    }

    double tr = A + D;
    double det = A * D - B * C;
    double S = sqrt( square(tr/2) - det );
    lambda(0) = tr/2 + S;
    lambda(1) = tr/2 - S;

    double SS = sqrt( max(square((A-D)/2) + B * C, 0.0) );
    if( A - D < 0 ) {
        v(0,0) = C;
        v(1,0) = - (A-D)/2 + SS;
        v(1,0) = + (A-D)/2 - SS;
        v(1,1) = B;
    } else {
        v(1,0) = C;
        v(1,1) = - (A-D)/2 - SS;
        v(0,0) = + (A-D)/2 + SS;
        v(1,0) = B;
    }

    v.col(0).normalize();
    v.col(1).normalize();

    theeta = atan(2*C/(A*D))/2;

    //double n1 = sqrt(square(v(0,0))+square(v(1,0)));
    //v(0,0) /= n1; v(1,0) /= n1;
    //double n2 = sqrt(square(v(1,0))+square(v(1,1)));
    //v(1,0) /= n2; v(1,1) /= n2;
}

#endif /* 2BY2EVD_HPP_ */
