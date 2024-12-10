#include <iostream>
#include <math.h>
#include <random>
#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fstream>


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

    std::cout << "( " << th << " , " << w << " )"<< std::endl;

  }

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

  float val;
  
  Sensor(): th(0), w(0) {}

  void set_data(Pendulum &p) {

    RandomVector R(0.0, 0.0, 0.09, 0.01, 0.3);
    R.set_vector();


    th = p.th + R.x;
    w = p.w + R.y;

  }
};
// sensor gives measurements Xt + W, so the jecobian is [1 0] [0 1]


std::ostream &operator<<(std::ostream &os, Pendulum p) {
  ///  return os << "{ th: " << p.th << " w: "  << p.w << "time: " << p.period * p.time << "}" << std::endl;
  return os << p.time * p.period << "\t" << p.th ;
}

std::ostream &operator<<(std::ostream &os, Sensor s) {

  return os << "\t" << s.th;
}


enum To {
  ToPredict,
  ToUpdate,
};

class KalmanFilter {
public:

  Eigen::Matrix<float, 2, 2> Jacobian_Model;
  Eigen::Matrix<float, 2, 2> Jacobian_Sensor;

  // RandomVector Sensor_Random = RandomVector(0.0, 0.0, 0.09, 0.01, 0.3);
  // RandomVector Model_Random = RandomVector(0.0, 0.0, 0.0, 0.0, 0.0);


  Eigen::Matrix<float, 2, 1> state;
  Eigen::Matrix<float, 2, 2> state_cov;
  

  Eigen::Matrix<float, 2, 1> buffer;
  Eigen::Matrix<float, 2, 2> bufferm;


  Eigen::Matrix<float, 2, 1> measurement;
  Eigen::Matrix<float, 2, 2> measurement_cov;

  Eigen::Matrix<float, 2, 2> V;
  Eigen::Matrix<float, 2, 2> W;

  int time = 0;
  float period = 0.1f;

  To todo;

  KalmanFilter() {
 
    //     ( 0.363005 , -1.24798 )
    //( -0.505593 , 0.297231 )
    state(0,0) = 0.505593f;
    state(1,0) = -0.297231f;

    state_cov = Eigen::Matrix<float, 2, 2>::Identity();
    measurement_cov = Eigen::Matrix<float, 2, 2>::Identity();

  // RandomVector Sensor_Random = RandomVector(0.0, 0.0, 0.09, 0.01, 0.3);

    //measurement_cov << 0.09, 0.1, 0.1, 0.3;


    W << 0.0, 0.0, 0.0, 0.0;
    V << 0.09, 0.01, 0.01, 0.3;

    // todo = ToPredict;

  };

  void popolate_model_jacobian() {
    Jacobian_Model << 1.0, period, (-period)*9.81*cos(state(0,0)), 1.0f;
  }

  void popolate_sensor_jacobian() {
    Jacobian_Sensor << 1.0f, 0.0f, 0.0f, 1.0f;
  }

  void predict() {
    //if (todo != ToPredict) return;

    popolate_model_jacobian();

    state(0,0) = state(0,0) + period*state(1,0);
    state(1,0) = state(1,0) - period*9.81*sin(state(0,0));


    // std::cout << "#####" << std::endl;
    // std::cout << Jacobian_Model << std::endl;
    // std::cout << "#####" << std::endl;



    state_cov = Jacobian_Model*state_cov*Jacobian_Model.transpose() + W;
    //state_cov = bufferm;
    todo = ToUpdate;
  }
  
  void update(Sensor s) {
    //    if (todo != ToUpdate) return;

    popolate_sensor_jacobian();


    Eigen::Matrix<float, 2, 1> result;
    result << s.th, s.w;

    Eigen::Matrix<float, 2, 1> ybar = result - state; // works only now because identity matrix

    Eigen::Matrix<float, 2, 2> S = (Jacobian_Sensor * state_cov * Jacobian_Sensor.transpose()) + V;

    Eigen::Matrix<float, 2, 2> K = state_cov * Jacobian_Sensor.transpose() * S.inverse();

    state = state + K*ybar;

    Eigen::Matrix<float, 2, 2> I;
    I << 1.0, 0.0, 0.0, 1.0;

    state_cov = (I - K*Jacobian_Sensor)*state_cov;

    todo = ToPredict;
  }
 

};

int main() {
  Pendulum P;
  Sensor S;
  KalmanFilter F;

 

  std::ofstream fs("data.txt", std::ios::out);

  fs << P ;

  for(int i = 0; i < 100 ; i++) {
    P.tick();
    F.predict();
    fs << P;
    S.set_data(P);
    fs << S;
    F.update(S);
    fs << "\t" << F.state(0,0);
    fs << std::endl;
  }



}
