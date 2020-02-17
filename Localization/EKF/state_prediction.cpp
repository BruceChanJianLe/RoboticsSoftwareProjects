#include <iostream>
#include <math.h>
#include <tuple>


using namespace std;

double new_mean, new_var;

tuple<double, double> statePrediction(double mean1, double var1, double mean2, double var2){
    // Calculate new mean and var
    double new_mean = mean1 + mean2;
    double new_var = var1 + var2;

    return make_tuple(new_mean, new_var);
}


int main(int argc, char ** argv){
    // Obtain the new mean and variance from the function
    tie(new_mean, new_var) = statePrediction(10.0, 4.0, 12.0, 4.0);
    cout << new_mean << " " << new_var << endl;

    return 0;
}