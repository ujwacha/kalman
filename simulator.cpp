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




class RandomVector {
public:
  
  float m1_, m2_, v1_, c12_, v2_;

  RandomVector(float m1, float m2, float v1, float c12, float v2) {
    m1_ = m1;
    m2_ = m2;
    v1_ = v1;
    v2_ = v2;
    c12_ = c12;
  }

  
  float l11, l22, l12;


  float x, y;

  void set_vector() {
    // Cholesky Decompositinn
    l11 = sqrt(v1_);
    l12 = c12_ / l11;

    if (v2_ - l12*l12 < 0) throw -1;

    l22 = sqrt(v2_ - l12*l12);

    float z1, z2;

    // Define random generator with Gaussian distribution
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();   
    std::default_random_engine generator(seed);
    std::normal_distribution<double> dist(0.0, 1.0);

    z1 = dist(generator);
    z2 = dist(generator);


    x = l11*z1;
    y = l12*z1 + l22*z2;

    x+= m1_;
    y+= m2_;
  }
  
};



class Sensor {
public:
  float th;
  float w;
  Sensor(): th(0), w(0) {}

  void set_data(Pendulum &p) {

    RandomVector R(0.0, 0.0, 0.01, 0.01, 0.09);
    R.set_vector();


    th = p.th + R.x;
    w = p.w + R.y;
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
  Sensor S;

  std::cout << P ;

  for(int i = 0; i < 100 ; i++) {
    P.tick();
    std::cout << P;
    S.set_data(P);
    std::cout << S << std::endl;
  }

}

