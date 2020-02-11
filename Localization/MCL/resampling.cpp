#include <iostream>
#include <vector>
#include <algorithm>


std::vector<double> w {0.6, 1.2, 2.4, 0.6, 1.2};

/**
 * @brief To compute the probabilities
 * @param w a vector of double
 * @return p probabilities
 */
std::vector<double> ComputeProb(std::vector<double> w){
    double sum = 0;
    std::vector<double> p(w.size());
    for(auto & single_w : w){
        sum += single_w;
    }
    std::transform(w.begin(),
                   w.end(),
                   p.begin(),
                   [sum](double w_single) -> double {
                       return w_single / sum;
                   });
    return p;
}

int main(int argc, char ** argv){
    // Print Probabilities each on a single line
    std::vector<double> p = ComputeProb(w);
    for(auto p_single : p){
        std::cout << p_single << " ";
    }
    std::cout << std::endl;
    
    return 0;
}
