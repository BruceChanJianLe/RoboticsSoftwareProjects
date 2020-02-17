#include <iostream>
#include <math.h>
#include <tuple>
#include <vector>


using namespace std;


double new_mean, new_var;

tuple<double, double> measurementUpdate(double m1, double v1, double m2, double v2){
    new_mean = (v2 * m1 + v1 * m2) / (v1 + v2);
    new_var = 1 / ( 1 / v1 + 1 / v2);
    return make_tuple(new_mean, new_var);
}

tuple<double, double> statePrediction(double m1, double v1, double m2, double v2){
    new_mean = m1 + m2;
    new_var = v1 + v2;
    return make_tuple(new_mean, new_var);
}


int main(int argc, char ** argv){
    // Measurements and measurement variance
    vector<double> measurement {5, 6, 7, 9, 10};
    double measurement_sig = 4;

    // Motion and motion variance
    vector<double> motion {1, 1, 2, 1, 1};
    double motion_sig = 2;

    // Initial state
    double mu = 0;
    double sig = 1;

    for(int i = 0; i < measurement.size(); i++){
        tie(mu, sig) = measurementUpdate(mu, sig, measurement[i], measurement_sig);
        cout << "Measurement Update, mu: " << mu << ", sig: " << sig << endl;
        tie(mu, sig) = statePrediction(mu, sig, motion[i], motion_sig);
        cout << "State Predition,    mu: " << mu << ", sig: " << sig << endl;
    }

    return 0;
}