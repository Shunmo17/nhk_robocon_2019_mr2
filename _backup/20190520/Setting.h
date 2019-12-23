// 歩行プログラムのソースファイルでインクルードする
//#pragma once

#ifndef _USETHREAD
#define _USETHREAD

#define PI 3.14159265358979

extern double eulersum[3];

//#define ODE 			// ODEでシミュレーションするときのみ定義する
#define CHANGE_OUTPUT // Output関数を置き換える
#define SERVO_LIMIT   // サーボの角度制限をかける

#ifdef ODE

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <cmath>
#include <iostream>

// 変数
extern double setangle[4][3];
extern double sensorangle[3];

// クラスの置き換え
// オイラー角
class Euler
{
  public:
	double roll = 0.0, pitch = 0.0, yaw = 0.0;
};

// ジャイロ
#define PB_7 27
#define PB_8 28
#define OPERATION_MODE_NDOF 0
extern int mode;
extern int modenum;
class BNO055
{
  public:
	Euler euler;
	BNO055(int s1, int s2){};
	void get_angles()
	{
		euler.roll = sensorangle[0] * 180 / M_PI;
		euler.pitch = sensorangle[1] * 180 / M_PI;
		euler.yaw = -sensorangle[2] * 180 / M_PI;
		//std::cout << "roll = " << (int)euler.roll << ":	pitch = " << (int)euler.pitch << ":	yaw = " << (int)euler.yaw << std::endl;
	};
	void setmode(double a){};
};

// パルス出力
/*
#define PB_2 0
#define PA_1 4
#define PB_0 8
#define PA_11 9
#define PC_8 1
#define PA_6 5
#define PA_7 6
#define PB_6 10
#define PB_5 14
#define PB_10 3
#define PA_10 7
#define PC_9 11
*/
class PwmOut
{
  public:
	int num[2] = {0, 0};
	void pulsewidth_us(double d){
		//setangle[num[0]][num[1]] = (d - 500) / (2000 / 270);
	};
	void period_ms(double ms){};
	PwmOut(int n)
	{
		num[0] = n % 4;
		num[1] = n % 3;
	};
	PwmOut(){};
};

// pc出力
#define USBTX 0
#define USBRX 1
class Serial
{
  public:
	Serial(double a, double b, double c){};
	void printf(char *s, double a = 1000, double b = 1000, double c = 1000){
		//if (a == 1000)std::cout << s << std::endl;
		//else if (b == 1000)std::cout << s << a << std::endl << std::endl;
		//else if (c == 1000)std::cout << s << a << "  " << b << std::endl << std::endl;
		//else std::cout << s << a << "  " << b << "  " << c << std::endl << std::endl;
	};
	void baud(double a){};
};

// ピン名
class PinName
{
  public:
};

// i2c
class I2C
{
  public:
	I2C(){};
	I2C(double a, double b){};
	I2C(PinName a, PinName b){};
	int write(double a, char *b, double c) { return (int)c; };
	void frequency(double a){};
};

// OLEDディスプレイ
class Stream
{
  public:
	Stream(){};
	Stream(double a, double b, double c){};
	void printf(char *s, double a = 1000, double b = 1000, double c = 1000){
		//if (a == 1000)std::cout << s << std::endl;
		//else if (b == 1000)std::cout << s << a << std::endl << std::endl;
		//else if (c == 1000)std::cout << s << a << "  " << b << std::endl << std::endl;
		//else std::cout << s << a << "  " << b << "  " << c << std::endl << std::endl;
	};
};

// タイマー割り込み
extern int Gyro;
extern int print_adjust;
class Ticker
{
  public:
	void attach(int *a, double b){};
};

// デジタル入力
#define USER_BUTTON 0
#define PH_1 81
#define PH_0 80
#define PC_15 315
#define PC_14 314
class DigitalIn
{
  public:
	int num = 0;
	int readnum = 0;
	DigitalIn(double a) { num = (int)a; };
	int read() { return readnum; };
};

// アナログ入力
#define PC_1 31
class AnalogIn
{
  public:
	AnalogIn(double a){};
	double read()
	{
		double dd = 15;
		double out = (800 / dd + 4) / (35 * 3.3);
		return out;
	};
};

// デジタル出力
#define PB_9 9
#define PB_3 3
class DigitalOut
{
  public:
	DigitalOut(double a){};
};

// 関数の置き換え
void wait(double);
void wait_ms(double);
void Wait_UB();

#endif

#endif