#include "Output.h"
#include "Common.h"

/**************************回転(角度で適当にやってるやつ)**********************************/
void Rotation_Deg(double x0, double y0, double all_height,
                  double rotation_angle, int rotation_times) //rotation_angleは回転角度[deg]
{
  Output_Angle(rf.s0.angle_old - 0.5 * rotation_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old - 0.5 * rotation_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old - 0.5 * rotation_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old - 0.5 * rotation_angle, lf.s1.angle_old, lf.s2.angle_old,
              10, 0.02);

  for (int i(1); i <= rotation_times; i++)
  {
    double height_angle(45);

    Output_Angle(rf.s0.angle_old + 1 * rotation_angle, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old + 1 * rotation_angle, rb.s1.angle_old - height_angle, rb.s2.angle_old,
                 lb.s0.angle_old + 1 * rotation_angle, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old + 1 * rotation_angle, lf.s1.angle_old - height_angle, lf.s2.angle_old,
                10, 0.02);

    Output_Angle(rf.s0.angle_old, rf.s1.angle_old, rf.s2.angle_old,
                 rb.s0.angle_old, rb.s1.angle_old + height_angle, rb.s2.angle_old,
                 lb.s0.angle_old, lb.s1.angle_old, lb.s2.angle_old,
                 lf.s0.angle_old, lf.s1.angle_old + height_angle, lf.s2.angle_old,
                10, 0.02);

    Output_Angle(rf.s0.angle_old - 1 * rotation_angle, rf.s1.angle_old - height_angle, rf.s2.angle_old,
                 rb.s0.angle_old - 1 * rotation_angle, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old - 1 * rotation_angle, lb.s1.angle_old - height_angle, lb.s2.angle_old,
                 lf.s0.angle_old - 1 * rotation_angle, lf.s1.angle_old, lf.s2.angle_old,
                10, 0.02);

    Output_Angle(rf.s0.angle_old, rf.s1.angle_old + height_angle, rf.s2.angle_old,
                 rb.s0.angle_old, rb.s1.angle_old, rb.s2.angle_old,
                 lb.s0.angle_old, lb.s1.angle_old + height_angle, lb.s2.angle_old,
                 lf.s0.angle_old, lf.s1.angle_old, lf.s2.angle_old,
                10, 0.02);
  }
  Output_Angle(rf.s0.angle_old + 0.5 * rotation_angle, rf.s1.angle_old, rf.s2.angle_old,
               rb.s0.angle_old + 0.5 * rotation_angle, rb.s1.angle_old, rb.s2.angle_old,
               lb.s0.angle_old + 0.5 * rotation_angle, lb.s1.angle_old, lb.s2.angle_old,
               lf.s0.angle_old + 0.5 * rotation_angle, lf.s1.angle_old, lf.s2.angle_old,
              10, 0.02);
}
