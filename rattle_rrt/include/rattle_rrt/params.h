#pragma once

#include <iostream>
using namespace std;

struct Params {
  /* System parameters.
  */
 public:
  double mass{0.0};
  double ixx{0.0};
  double iyy{0.0};
  double izz{0.0};
  double ixy{0.0};
  double iyz{0.0};
  double ixz{0.0};
  double cx{0.0};
  double cy{0.0};
  double cz{0.0};
  double sigma[4] = {0.0};

  Params(double mass, double ixx, double iyy, double izz, double ixy, double iyz, double ixz, double cx, double cy, double cz, double Cov[4]):
    mass(mass), ixx(ixx), iyy(iyy), izz(izz), ixy(ixy),
    iyz(iyz), ixz(ixz), cx(cx), cy(cy), cz(cz) {    
      std::copy(Cov, Cov + 5, this->sigma);
  }

  Params(double mass, double ixx, double iyy, double izz, double ixy, double iyz, double ixz, double cx, double cy, double cz):
    mass(mass), ixx(ixx), iyy(iyy), izz(izz), ixy(ixy),
    iyz(iyz), ixz(ixz), cx(cx), cy(cy), cz(cz) {    
  }

  Params(double mass, double ixx, double iyy, double izz):
    mass(mass), ixx(ixx), iyy(iyy), izz(izz) { 
  }

  Params() {
  }

  friend ostream& operator<<(ostream& os, const Params& params) {
      os << "mass: " << params.mass << " ixx: " << params.ixx << " iyy: " << params.iyy <<
      " izz: " << params.izz << " ixy: " << params.ixy << " iyz: " << params.iyz << " ixz: " <<
      params.ixz << " cx: " << params.cx << " cy: " << params.cy << " cz: " << params.cz;
      return os;
  }
};