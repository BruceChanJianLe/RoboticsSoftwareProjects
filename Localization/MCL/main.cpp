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
    Robot p[1000];
    for(auto & single_p : p){
        single_p.set_noise(0.05, 0.05, 5.0);
        // std::cout << single_p.show_pose() << std::endl;
    }

    // Re-initialize myrobot object and Initialize a measurement vector
    myrobot = Robot();
    std::vector<double> z;
    myrobot = myrobot.move(0.1, 5.0);
    z = myrobot.sense();

    // Simulate motion fo reach particle
    Robot p2[1000];
  /*int i = 0;
    for(auto single_p2 : p2){
        single_p2 = p[i].move(0.1, 5.0);
        p[i] = single_p2;
        std::cout << i << " " << p[i].show_pose() << std::endl;
        ++i;
    }
*/

    for(int i = 0; i < sizeof(p2) / sizeof(p2[0]); i++){
        p2[i] = p[i].move(0.1, 5.0);
        p[i] = p2[i];
        // std::cout << i << " " << p[i].show_pose() << std::endl;
    }

    double w[1000];
    for(int i = 0; i < sizeof(w) / sizeof(w[0]); i++){
        w[i] = p[i].measurement_prob(z);
        std::cout << i << " " << w[i] << std::endl;
    }

    return 0;
}
