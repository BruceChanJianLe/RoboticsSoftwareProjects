#include <iostream>
#include <math.h>
#include <tuple>
#include "Core"
#include "LU"


using namespace std;
using namespace Eigen;


vector<float> measurement {1, 2, 3};

tuple<MatrixXf, MatrixXf> kalmanfilter(MatrixXf)