#include "mcl.h"


Robot::Robot(){
    // Constructor
    x = gen_real_random() * world_size; // robot's x coordinate
    y = gen_real_random() * world_size; // robot's y coordinate
    orient = gen_real_random() * 2.0 * M_PI; // robot's orientation

    forward_noise = 0.0; //noise of the forward movement
    turn_noise = 0.0; //noise of the turn
    sense_noise = 0.0; //noise of the sensing
}


void Robot::set(double new_x, double new_y, double new_orient){
    // Set robot new position and orientation
    if (new_x < 0 || new_x >= world_size){
        throw std::invalid_argument("X coordinate out of bound!");
    } if(new_y < 0 || new_y >= world_size){
        throw std::invalid_argument("X coordinate out of bound!");
    } if(new_orient < 0 || new_orient >= 2 * M_PI){
        throw std::invalid_argument("Orientation must be in [0..2pi]");
    }

    x = new_x;
    y = new_y;
    orient = new_orient;
}


void Robot::set_noise(double new_forward_noise, double new_turn_noise, double new_sensor_noise){
    // Simulate noise to use in particle filter
    forward_noise = new_forward_noise;
    turn_noise = new_turn_noise;
    sense_noise = new_sensor_noise;
}


std::vector<double> Robot::sense(){
    // Measure the distance from the robot toward the 8 landmarks
    std::vector<double> z(sizeof(landmarks)/sizeof(landmarks[0]));
    double dist;

    for(int i = 0; i < sizeof(landmarks)/sizeof(landmarks[0]); i++){
        dist = std::sqrt(std::pow((x - landmarks[i][0]), 2) + std::pow((y - landmarks[i][1]), 2));
        dist += gen_gauss_random(0.0, sense_noise);
        z[i] = dist;
    }
    return z;
}

