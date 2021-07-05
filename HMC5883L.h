
#ifndef HMC5883L_H
#define HMC5883L_H

  extern float actual_compass_heading;
  
  void init_HMC5883L();
  void test_HMC5883L();
  void read_HMC5883L();

  void HMC5883L_calculator();

#endif
