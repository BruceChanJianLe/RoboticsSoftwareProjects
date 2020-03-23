#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>


using namespace std;
using namespace Eigen;


vector<float> measurement {1, 2, 3};

tuple<MatrixXf, MatrixXf> kalmanFilter(MatrixXf x,
                                       MatrixXf P,
                                       MatrixXf u,
                                       MatrixXf F,
                                       MatrixXf H,
                                       MatrixXf R,
                                       MatrixXf I)
{
     for(int n = 0; n < measurement.size(); n++)
     {
          // Kalman Filter function
          
          // Measurement Update
          MatrixXf Z(1, 1);
          Z << measurement[n];

          MatrixXf y(1, 1);
          y << Z - (H * x);

          MatrixXf S(1, 1);
          S << H * P * H.transpose() + R;

          // Calculate Kalman Gain
          MatrixXf K(2, 1);
          K << P * H.transpose() * S.inverse();

          // Calculate posterior state and covariance
          x << x + K * y;
          P << (I - K * H) * P;

          // State prediction
          x << F * x + u;
          P << F * P * F.transpose();
     }
     return make_tuple(x, P);
}


int main(int argc, char ** argv)
{
    MatrixXf x(2, 1);   // Initial state (location and velocity)
    x << 0,
         0;

    MatrixXf P(2, 2);   // Initial Uncertainty
    P << 100, 0,
         0, 100;

    MatrixXf u(2, 1);   // External Motion
    u << 0,
         0;

    MatrixXf F(2, 2);   // Next State Function
    F << 1, 1,
         0, 1;

    MatrixXf H(1, 2);   // Measurement Function
    H << 1,
         0;

    MatrixXf R(1, 1);   // Measurement Uncertainty
    R << 1;

    MatrixXf I(2, 2);   // Identity Matrix
    I << 1, 0,
         0, 1;

    tie(x, P) = kalmanFilter(x, P, u, F, H, R, I);
    cout << "x = \n" << x << endl;
    cout << "P = \n" << P << endl;

    return 0;
}