#include "include/mcl.h"


int main(int argc, char ** argv){
    std::cout <<"I am ready for coding the MCL!\n";

    // Instatiating a robto object from the Robot class
    Robot myrobot;
    myrobot.set_noise(5.0, 0.1, 5.0);
    myrobot.set(30.0, 50.0, M_PI / 2.0);
    myrobot.move(-M_PI / 2.0, 15.0);
    std::cout << myrobot.read_sensors() << std::endl;
    myrobot.move(-M_PI / 2.0, 10.0);
    std::cout << myrobot.read_sensors() << std::endl;

    // Instatiating 1000 particles each with a random position and orientation
    int n = 1000;
    Robot p[n];
    for(auto & single_p : p){
        single_p.set_noise(0.05, 0.05, 5.0);
    }

    // Re-initialize myrobot object and Initialize a measurement vector
    myrobot = Robot();
    std::vector<double> z;
    myrobot = myrobot.move(0.1, 5.0);
    z = myrobot.sense();

    // Simulate motion fo reach particle
    Robot p2[n];

    for(int i = 0; i < sizeof(p2) / sizeof(p2[0]); i++){
        p2[i] = p[i].move(0.1, 5.0);
        p[i] = p2[i];
    }

    double w[n];
    for(int i = 0; i < sizeof(w) / sizeof(w[0]); i++){
        w[i] = p[i].measurement_prob(z);
        std::cout << i << " " << w[i] << std::endl;
    }

    Robot p3[n];
    int idx = gen_real_random() * n; // 1000 is the number of particles
    double beta = 0.0;
    double mw = max(w, n);
    for(int i = 0; i < n; i++){
        beta += gen_real_random() * 2.0 * mw;
        while(beta > w[idx]){
            beta -= w[idx];
            idx = mod((idx + 1), n);
        }
        p3[i] = p[idx];
    }
    for(int i = 0; i < n; i++){
        p[i] = p3[i];
        std::cout << p[i].show_pose() << std::endl;
    }
    return 0;
}
