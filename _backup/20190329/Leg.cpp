#include "Leg.h"

#define PI 3.14159265358979

double r0(0.070), r1(0.14301), r2(0.29795); //リンクの長さ[m] 288.95+9=297.95mm

void Leg::calc(double x, double y, double z, double height, double &angle0, double &angle1, double &angle2)
{
	double d;
	d = sqrt(pow(x, 2) + pow(y, 2)) - r0;
	double hz = height - z;
	angle0 = (360 / (2 * PI)) * atan(y / x);
	double temp = sqrt(d * d + hz * hz);
	if (temp > r1 + r2) {
		d *= (r1 + r2) / temp / 1.01;
		hz *= (r1 + r2) / temp / 1.01;
#ifdef ODE
		std::cout << "position_error outside" << std::endl;	// 指定した座標が遠すぎ
#endif
	}
	else if (temp < abs(r1 - r2)) {
		d *= abs(r1 - r2)*1.01 / temp;
		hz *= abs(r1 - r2)*1.01 / temp;
#ifdef ODE
		std::cout << "position_error inside" << std::endl;	// 指定した座標が近すぎ
#endif
	}
	angle1 = -(360 / (2 * PI)) * (acos((pow(r1, 2) + pow(hz, 2) + pow(d, 2) - pow(r2, 2)) / (2 * r1 * sqrt(pow(hz, 2) + pow(d, 2)))) - atan((hz) / d));
	angle2 = 180 - (360 / (2 * PI)) * acos((pow(r1, 2) + pow(r2, 2) - pow(hz, 2) - pow(d, 2)) / (2 * r1 * r2));
}

Leg::Leg(Servo &_s0, Servo &_s1, Servo &_s2) : s0(_s0), s1(_s1), s2(_s2)
{
  theta0 = theta1 = theta2 = 0.0;
  x = y = z = 0.0;
  adjust = 0.0;
}