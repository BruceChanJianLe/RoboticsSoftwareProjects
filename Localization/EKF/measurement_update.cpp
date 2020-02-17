#include <iostream>
#include <math.h>
#include <tuple>


using namespace std;


double new_mean, new_var;

tuple<double, double> measurementUpdate(double mean1, double var1, double mean2, double var2){
    // Calculate new mean
    double new_mean = (var2 * mean1 + var1 * mean2) / (var2 + var1);
    double new_var = 1 / (1 / var1 + 1 / var2);
    return make_tuple(new_mean, new_var);
}


int main(int argc, char ** argv){
    tie(new_mean, new_var) = measurementUpdate(10.0, 8.0, 13.0, 2.0);
    cout << new_mean << " " << new_var << endl;
    
    return 0;
}