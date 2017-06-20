#ifndef FILTER_H
#define FILTER_H

#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

class Filter {
  public:
    Filter();
    int update(int d_angle,int radius);
    int getSpeed();
  private:
    int sensed_strength;
    int speed;
};
#endif
