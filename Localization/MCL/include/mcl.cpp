#include "include/mcl.h"


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


Robot Robot::move(double turn, double forward){
    if(forward < 0){
        throw std::invalid_argument("Robot cannot move backwards!");
    }
    // Add randomness while turning
    orient = orient + turn + gen_gauss_random(0.0, turn_noise);
    orient = mod(orient, 2 * M_PI);

    // Add randomness while moving forward
    double dist = forward + gen_gauss_random(0.0, forward_noise);
    x = x + (std::cos(orient) * dist);
    y = y + (std::sin(orient) * dist);

    // Because the world is cyclic
    x = mod(x, world_size);
    y = mod(y, world_size);

    // Set partiascle
    Robot res;
    res.set(x, y, orient);
    res.set_noise(forward_noise, turn_noise, sense_noise);

    return res;
}


std::string Robot::show_pose(){
    // Returns the robot current position and orientation in a string format
    return "[x=" + std::to_string(x) = " y=" + std::to_string(y) + " orient=" + std::to_string(orient) + "]";
}


std::string Robot::read_sensors(){
    // Return all the distance from the robot towards the landmarks
    std::vector<double> z = sense();
    std::string readings = "[";
    for(int i = 0; i < z.size(); i++){
        readings += std::to_string(z[i]) + ' ';
    }
    readings[readings.size() - 1] = ']';

    return readings;
}


double Robot::measurement_prob(std::vector<double> measurement){
    // Calculates how likely a measurement should be
    double prob = 1.0;
    double dist;

    for(int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++){
        dist = std::sqrt(std::pow((x - landmarks[i][0]), 2) + std::pow((y - landmarks[i][1]), 2));
        prob *= gaussian(dist, sense_noise, measurement[i]);
    }

    return prob;
}


double Robot::gen_gauss_random(double mean, double variance){
    // Random gaussian distribution
    std::normal_distribution<double> gauss_dist(mean, variance);
    return gauss_dist(gen);
}


double Robot::gaussian(double mu, double sigma, double x){
    // Probability of x for 1 dimension gaussian distribution with mean mu and variance sigma
    return std::exp(-std::pow((mu - x), 2) / (sigma * sigma) / 2.0) / std::sqrt(2.0 * M_PI * sigma * sigma);
}


double gen_real_random(){
    // Generate real random number between 0 and 1
    std::uniform_real_distribution<double> real_dist(0.0, 1.0);
    return real_dist(gen);
}


double mod(double first_term, double second_term){
    // Compute the modulus
    return first_term - (second_term) * std::floor(first_term / second_term);
}


double evaluation(Robot r, Robot p[], int n){
    // Calculate the mean error of the system
    double sum = 0.0;
    for(int i = 0; i <n ;i++){
        // The second part is because the world is cyclic
        double dx = mod((p[i].x - r.x + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double dy = mod((p[i].y - r.y + (world_size / 2.0)), world_size) - (world_size / 2.0);
        double err = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        sum += err;
    }
    return sum / n;
}


double max(double arr[], int n){
    // Identity the max element in an array
    double max = 0;
    for(int i = 0; i < n; i++){
        if(arr[i] > max){
            max = arr[i];
        }
    }
    return max;
}


void visualization(int n, Robot robot, int step, Robot p[], Robot pr[]){
    // Draw the robot, landmarks, particles and resampled particles on a graph

    // Graph Format
    plt::title("MCL, step " + std::to_string(step));
    plt::xlim(0, 100);
    plt::ylim(0, 100);

    // Draw particles in green
    for(int i = 0; i < n; i++){
        plt::plot({p[i].x}, {p[i].y}, "go");
    }

    // Draw resampled particles in yellow
    for(int i = 0; i < n; i++){
        plt::plot({pr[i].x}, {pr[i].y}, "yo");
    }

    // Draw landmarks in red
    for(int i = 0; i < sizeof(landmarks) / sizeof(landmarks[0]); i++){
        plt::plot({landmarks[i][0]}, {landmarks[i][1]}, "ro");
    }

    // Draw robot position in blue
    plt::plot({robot.x}, {robot.y}, "bo");

    // Save the image and close the plot
    plt::save("./Image/Step" + std::to_string(step) + ".png");
    plt::clf();
}