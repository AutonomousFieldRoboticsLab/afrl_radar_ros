#pragma once

#include <cstdint>
#include <vector>

struct USR_Data {
  unsigned short left_front;
  unsigned short right_front;
};

struct MWR_Data {
  int index;
  float Range;
  float RadialVelocity;
  float RadialAcc;
  float Azimuth;
  float Power;
  MWR_Data(int i, float Rg, float RV, float RA, float Az, float Pw)
      : index(i), Range(Rg), RadialVelocity(RV), RadialAcc(RA), Azimuth(Az),
        Power(Pw) {}
};
class USRadar {
public:
  USRadar();
  ~USRadar();
  bool Read(USR_Data &data);
};
class MWRadar {
public:
  MWRadar();
  ~MWRadar();
  bool Read(std::vector<MWR_Data> &data, int num = 1);
};
