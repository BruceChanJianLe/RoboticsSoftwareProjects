#include <iostream>
#include <math.h>


using namespace std;


double f(double mu, double sigma2, double x){
    return (1 / sqrt( 2 * M_PI * sigma2 * exp((x - mu) * (x - mu) / sigma2)));
}


int main(int argc, char ** argv){
    cout << f(10.0, 4.0, 8.0) << endl;
    return 0;
}