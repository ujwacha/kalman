#include <iostream>
#include <math.h>
#include <random>
#include <chrono>


class Pendulum {
public:

  // state
  float th;
  float w = 0.0f;
  // necessary for calculation
  int time = 0;
  float period = 0.1f;

  Pendulum() {
    th = 0.5f;
  }


  void tick() {
    time ++;
    th = th + period*w;
    w = w - period*(9.81/1)*sin(th);
  }


  // (float, float) get_sensor_data() {
  //   return (th + dist(generator), w + dist(generator));
  // }

};

class Sensor {
public:
  float th;
  float w;
  const double mean;
  const double stddev; 

  Sensor(double sd): mean(0.0), stddev(sd) {
    
  }

  void set_data(Pendulum &p) {
     // random number, gaussian noise
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();   // Define random generator with Gaussian distribution
    std::default_random_engine generator(seed);
    std::normal_distribution<double> dist(mean, stddev);

    th = p.th + dist(generator);
    w = p.w + dist(generator);
    
  }
};


std::ostream &operator<<(std::ostream &os, Pendulum p) {
  ///  return os << "{ th: " << p.th << " w: "  << p.w << "time: " << p.period * p.time << "}" << std::endl;
  return os << p.time * p.period << "\t" << p.th ;
}

std::ostream &operator<<(std::ostream &os, Sensor s) {

  return os << "\t" << s.th;
}
  

int main() {
  Pendulum P;
  Sensor S(0.2);

  std::cout << P ;

  for(int i = 0; i < 100 ; i++) {
    P.tick();
    std::cout << P;
    S.set_data(P);
    std::cout << S << std::endl;
  }

}


















