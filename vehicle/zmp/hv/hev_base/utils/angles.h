#ifndef __ANGLES_H__
#define __ANGLES_H__

#define ANGLE_2PI 2.0*M_PI
 
class Angle
{
public:

  /** difference will be in [-PI PI]
    angles must be in radians (any standard) */
  static double norm(double a);
  static double diff(double a, double b);
  static double add(double a, double b);
  static double R2D(double angle_rad);
  static double D2R(double angle_deg);
 
};

#endif //__ANGLES_H__
