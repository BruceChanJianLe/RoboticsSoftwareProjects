#ifndef MCL_H
#define MCL_H


#include <iostream>
#include <string>
#include <math.h>
#include <vector>
#include <stdexcept>    // throw errors
#include <random>
#include "matplotlibcpp.h"


namespace plt = matplotlibcpp;

// Define Landmarks
double landmarks[8][2] {{20.0, 20.0}, {20.0, 80.0}, {20.0, 50.0},
                       {50.0, 20.0}, {50.0, 80.0}, {80.0, 80.0},
                       {80.0, 20.0}, {80.0, 50.0}};

// Map size (in meters)
double world_size = 100.0;

// Robot Class
class Robot{
    public:
        Robot();

        void set(double new_x, double new_y, double new_orientation);
        void set_noise(double new_forward_noise, double new_turn_noise, double new_sense_noise);
        std::vector<double> sense();
        Robot move(double turn, double forward);
        std::string show_pose();
        std::string read_sensors();
        double measurement_prob(std::vector<double> measurement);
        double x, y, orient;    // Robot pose
        double forward_noise, turn_noise, sense_noise;  // Robot's noise

    private:
        double gen_gauss_random(double mean, double variance);
        double gaussian(double mu, double sigma, double x);
};

// Global Functions
double mod(double first_term, double second_term);
double gen_real_random();
double evaluation(Robot r, Robot p[], int n);
double max(double arr[], int n);
void visualization(int n, Robot robot, int step, Robot p[], Robot pr[]);

// Random Generators
std::random_device rd;
std::mt19937 gen(rd());

#endif
