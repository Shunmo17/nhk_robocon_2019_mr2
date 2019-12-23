#include "Common.h"
#include "PCA9685.h"
#include "Function.h"
#include "math.h"
#include "Display.h"
#include "Gyro_sensor.h"

#define KP_ROLL 0.001
#define KP_PITCH 0.0
#define KP_YAW 0.0
//4/3はじめのパラメータ：0.1, 0.1, 0.05

//#define Z_ADJUST	// 段差用のz座標調節を入れる

// 外部変数
double eulersum[3] = {0, 0, 0};											 // 本体に対する脚の角度
double bodyangle[3] = {0, 0, 0};										 // 本体の角度
double relativepos[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // 脚の相対座標
double legposition[4][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0, 0, 0}}; // 脚の絶対座標
double bodyposition[3] = {0, 0, 0};										 // 本体の絶対座標
int legflag[4] = {0, 0, 0, 0};											 // 接地している脚のフラグ
int redrawflag = 0;														 // opengl描画用のフラグ

/*****************************サーボドライバ*****************************/

/*
int SetPos(double angle)
{
	double pos(0.0);
	double pulse(0.0);
	pos = (2500 - 500) / 270 * angle + 500;
	pulse = (pos / 20000) * 4096 - 1;
	return (int)pulse;
}*/

#ifdef ODE
// 行列の表示
void showmatrix(double matrix[9])
{
	std::cout << "|" << matrix[0] << " 	" << matrix[1] << "		" << matrix[2] << "|" << std::endl
			  << "|" << matrix[3] << "	" << matrix[4] << "		" << matrix[5] << "|" << std::endl
			  << "|" << matrix[6] << "	" << matrix[7] << "		" << matrix[8] << "|" << std::endl
			  << std::endl;
}
#endif

// 3×3行列の乗算
void calcmatrix3(double solution[9], double a[9], double b[9])
{
	for (int i = 0; i < 9; i++)
	{
		solution[i] = 0;
	}
	for (int i = 0; i < 9; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			solution[i] += a[(i / 3) * 3 + j] * b[i % 3 + 3 * j];
		}
	}
}

// 回転行列の設定
void rotationmatrix(double matrix3[9], double roll_, double pitch_, double yaw_)
{
	double theta[3] = {roll_ * PI / 180.0, pitch_ * PI / 180.0, yaw_ * PI / 180.0};

	double rotx[9] = {1, 0, 0,
					  0, cos(theta[0]), -sin(theta[0]),
					  0, sin(theta[0]), cos(theta[0])};
	double roty[9] = {cos(theta[1]), 0, sin(theta[1]),
					  0, 1, 0,
					  -sin(theta[1]), 0, cos(theta[1])};
	double rotz[9] = {cos(theta[2]), -sin(theta[2]), 0,
					  sin(theta[2]), cos(theta[2]), 0,
					  0, 0, 1};

	double rottemp[9];
	calcmatrix3(rottemp, roty, rotz);
	calcmatrix3(matrix3, rotx, rottemp); // x->y->z型オイラー角
}

// 座標の回転
void rotatecoordinate(double matrix3_[], double *xx, double *yy, double *zz)
{
	double pos[3] = {*xx, *yy, *zz};
	*xx = matrix3_[0] * pos[0] + matrix3_[1] * pos[1] + matrix3_[2] * pos[2];
	*yy = matrix3_[3] * pos[0] + matrix3_[4] * pos[1] + matrix3_[5] * pos[2];
	*zz = matrix3_[6] * pos[0] + matrix3_[7] * pos[1] + matrix3_[8] * pos[2];
}

// 逆運動学,"Output.cpp"の置き換え
/*****************************ステップ出力関数(角度指定)[いきなり目標値]***************************/

void Output_Angle(double angle_a_s0, double angle_a_s1, double angle_a_s2,
				  double angle_b_s0, double angle_b_s1, double angle_b_s2,
				  double angle_c_s0, double angle_c_s1, double angle_c_s2,
				  double angle_d_s0, double angle_d_s1, double angle_d_s2,
				  int step_times, double wait_time)
//step_timesは使ってない
//wait_timeは使ってない
//step_timesを0にするとステップなしで即座にangle_refを出力

{
	rf.s0.angle_ref = angle_a_s0;
	rf.s1.angle_ref = angle_a_s1;
	rf.s2.angle_ref = angle_a_s2;
	rb.s0.angle_ref = angle_b_s0;
	rb.s1.angle_ref = angle_b_s1;
	rb.s2.angle_ref = angle_b_s2;
	lb.s0.angle_ref = angle_c_s0;
	lb.s1.angle_ref = angle_c_s1;
	lb.s2.angle_ref = angle_c_s2;
	lf.s0.angle_ref = angle_d_s0;
	lf.s1.angle_ref = angle_d_s1;
	lf.s2.angle_ref = angle_d_s2;

	setServoPulse(0, rf.s0.angle(rf.s0.angle_ref));
	setServoPulse(1, rf.s1.angle(rf.s1.angle_ref));
	setServoPulse(2, rf.s2.angle(rf.s2.angle_ref));
	setServoPulse(3, rb.s0.angle(-rb.s0.angle_ref));
	setServoPulse(4, rb.s1.angle(rb.s1.angle_ref));
	setServoPulse(5, rb.s2.angle(rb.s2.angle_ref));
	setServoPulse(6, lb.s0.angle(lb.s0.angle_ref));
	setServoPulse(7, lb.s1.angle(lb.s1.angle_ref));
	setServoPulse(8, lb.s2.angle(lb.s2.angle_ref));
	setServoPulse(9, lf.s0.angle(-lf.s0.angle_ref));
	setServoPulse(10, lf.s1.angle(lf.s1.angle_ref));
	setServoPulse(11, lf.s2.angle(lf.s2.angle_ref));

	rf.s0.angle_old = rf.s0.angle_ref;
	rf.s1.angle_old = rf.s1.angle_ref;
	rf.s2.angle_old = rf.s2.angle_ref;
	rb.s0.angle_old = rb.s0.angle_ref;
	rb.s1.angle_old = rb.s1.angle_ref;
	rb.s2.angle_old = rb.s2.angle_ref;
	lb.s0.angle_old = lb.s0.angle_ref;
	lb.s1.angle_old = lb.s1.angle_ref;
	lb.s2.angle_old = lb.s2.angle_ref;
	lf.s0.angle_old = lf.s0.angle_ref;
	lf.s1.angle_old = lf.s1.angle_ref;
	lf.s2.angle_old = lf.s2.angle_ref;
}

/**************************ステップ出力関数(角度指定)[徐々に目標値]********************/
void Output_Step_Angle(double angle_a_s0, double angle_a_s1, double angle_a_s2,
					   double angle_b_s0, double angle_b_s1, double angle_b_s2,
					   double angle_c_s0, double angle_c_s1, double angle_c_s2,
					   double angle_d_s0, double angle_d_s1, double angle_d_s2,
					   int step_times, double wait_time)
//step_timesは使ってない
//wait_timeは使ってない
//step_timesを0にするとステップなしで即座にangle_refを出力

{
	rf.s0.angle_ref = angle_a_s0;
	rf.s1.angle_ref = angle_a_s1;
	rf.s2.angle_ref = angle_a_s2;
	rb.s0.angle_ref = angle_b_s0;
	rb.s1.angle_ref = angle_b_s1;
	rb.s2.angle_ref = angle_b_s2;
	lb.s0.angle_ref = angle_c_s0;
	lb.s1.angle_ref = angle_c_s1;
	lb.s2.angle_ref = angle_c_s2;
	lf.s0.angle_ref = angle_d_s0;
	lf.s1.angle_ref = angle_d_s1;
	lf.s2.angle_ref = angle_d_s2;

	for (int i(1); i <= step_times; i++)
	{
		rf.s0.angle_now = (rf.s0.angle_ref - rf.s0.angle_old) * ((double)i / (double)step_times) + rf.s0.angle_old;
		rf.s1.angle_now = (rf.s1.angle_ref - rf.s1.angle_old) * ((double)i / (double)step_times) + rf.s1.angle_old;
		rf.s2.angle_now = (rf.s2.angle_ref - rf.s2.angle_old) * ((double)i / (double)step_times) + rf.s2.angle_old;
		rb.s0.angle_now = (rb.s0.angle_ref - rb.s0.angle_old) * ((double)i / (double)step_times) + rb.s0.angle_old;
		rb.s1.angle_now = (rb.s1.angle_ref - rb.s1.angle_old) * ((double)i / (double)step_times) + rb.s1.angle_old;
		rb.s2.angle_now = (rb.s2.angle_ref - rb.s2.angle_old) * ((double)i / (double)step_times) + rb.s2.angle_old;
		lb.s0.angle_now = (lb.s0.angle_ref - lb.s0.angle_old) * ((double)i / (double)step_times) + lb.s0.angle_old;
		lb.s1.angle_now = (lb.s1.angle_ref - lb.s1.angle_old) * ((double)i / (double)step_times) + lb.s1.angle_old;
		lb.s2.angle_now = (lb.s2.angle_ref - lb.s2.angle_old) * ((double)i / (double)step_times) + lb.s2.angle_old;
		lf.s0.angle_now = (lf.s0.angle_ref - lf.s0.angle_old) * ((double)i / (double)step_times) + lf.s0.angle_old;
		lf.s1.angle_now = (lf.s1.angle_ref - lf.s1.angle_old) * ((double)i / (double)step_times) + lf.s1.angle_old;
		lf.s2.angle_now = (lf.s2.angle_ref - lf.s2.angle_old) * ((double)i / (double)step_times) + lf.s2.angle_old;

		setServoPulse(0, rf.s0.angle(rf.s0.angle_now));
		setServoPulse(1, rf.s1.angle(rf.s1.angle_now));
		setServoPulse(2, rf.s2.angle(rf.s2.angle_now));
		setServoPulse(3, rb.s0.angle(-rb.s0.angle_now));
		setServoPulse(4, rb.s1.angle(rb.s1.angle_now));
		setServoPulse(5, rb.s2.angle(rb.s2.angle_now));
		setServoPulse(6, lb.s0.angle(lb.s0.angle_now));
		setServoPulse(7, lb.s1.angle(lb.s1.angle_now));
		setServoPulse(8, lb.s2.angle(lb.s2.angle_now));
		setServoPulse(9, lf.s0.angle(-lf.s0.angle_now));
		setServoPulse(10, lf.s1.angle(lf.s1.angle_now));
		setServoPulse(11, lf.s2.angle(lf.s2.angle_now));

		wait(wait_time);
	}

	setServoPulse(0, rf.s0.angle(rf.s0.angle_ref));
	setServoPulse(1, rf.s1.angle(rf.s1.angle_ref));
	setServoPulse(2, rf.s2.angle(rf.s2.angle_ref));
	setServoPulse(3, rb.s0.angle(-rb.s0.angle_ref));
	setServoPulse(4, rb.s1.angle(rb.s1.angle_ref));
	setServoPulse(5, rb.s2.angle(rb.s2.angle_ref));
	setServoPulse(6, lb.s0.angle(lb.s0.angle_ref));
	setServoPulse(7, lb.s1.angle(lb.s1.angle_ref));
	setServoPulse(8, lb.s2.angle(lb.s2.angle_ref));
	setServoPulse(9, lf.s0.angle(-lf.s0.angle_ref));
	setServoPulse(10, lf.s1.angle(lf.s1.angle_ref));
	setServoPulse(11, lf.s2.angle(lf.s2.angle_ref));

	rf.s0.angle_old = rf.s0.angle_ref;
	rf.s1.angle_old = rf.s1.angle_ref;
	rf.s2.angle_old = rf.s2.angle_ref;
	rb.s0.angle_old = rb.s0.angle_ref;
	rb.s1.angle_old = rb.s1.angle_ref;
	rb.s2.angle_old = rb.s2.angle_ref;
	lb.s0.angle_old = lb.s0.angle_ref;
	lb.s1.angle_old = lb.s1.angle_ref;
	lb.s2.angle_old = lb.s2.angle_ref;
	lf.s0.angle_old = lf.s0.angle_ref;
	lf.s1.angle_old = lf.s1.angle_ref;
	lf.s2.angle_old = lf.s2.angle_ref;
}

/**************************ステップ出力関数(足先座標指定)***********************/
void Output_Coordinate(double x_a, double y_a, double z_a,
					   double x_b, double y_b, double z_b,
					   double x_c, double y_c, double z_c,
					   double x_d, double y_d, double z_d,
					   double all_height,
					   int step_times, double wait_time)
{
	double posnow[4][3], all__height;
	double temp = 0.1901 / 2; // 胴体の幅の半分[m]
	static double rotmatrix3[9];

	int deltastep; // 一定時間置きを計測するための変数
	if (wait_time == 0)
		deltastep = 2;
	else
		deltastep = int(0.5 / (wait_time));
	if (deltastep == 0)
		deltastep = 2;

	// 脚から見た座標に設定するとき用
	x_a = x_a + temp;
	y_a = y_a + temp;
	z_a = z_a;
	x_b = x_b + temp;
	y_b = -y_b - temp;
	z_b = z_b;
	x_c = -x_c - temp;
	y_c = -y_c - temp;
	z_c = z_c;
	x_d = -x_d - temp;
	y_d = y_d + temp;
	z_d = z_d;

	// 前回の足先の位置
	static double oldlegpos[4][3] = {{temp * 3, temp * 3, 0},
									 {temp * 3, -temp * 3, 0},
									 {-temp * 3, -temp * 3, 0},
									 {-temp * 3, temp * 3, 0}};

	// 設置している脚を確認
	int num = 0; // 脚の数
	for (int j = 0; j < 4; j++)
		legflag[j] = 0;
	if (oldlegpos[0][2] == 0)
	{
		if (z_a == 0)
		{
			legflag[0] = 1;
			num++;
		}
	}
	if (oldlegpos[1][2] == 0)
	{
		if (z_b == 0)
		{
			legflag[1] = 1;
			num++;
		}
	}
	if (oldlegpos[2][2] == 0)
	{
		if (z_c == 0)
		{
			legflag[2] = 1;
			num++;
		}
	}
	if (oldlegpos[3][2] == 0)
	{
		if (z_d == 0)
		{
			legflag[3] = 1;
			num++;
		}
	}

	for (int i(1); i <= step_times + 1; i++)
	{
		//bno.setmode(OPERATION_MODE_NDOF);
		bno.get_angles();

		// 回転座標を設定
		double angleKp[3] = {KP_ROLL, KP_PITCH, KP_YAW}; // 姿勢補正のP制御のゲイン,(ロール角,ピッチ角,ヨー角)

		double euler3param[3];
		euler3param[0] = bno.euler.roll - roll_offset;
		euler3param[1] = bno.euler.pitch - pitch_offset;
		double euler3param2 = bno.euler.yaw - yaw_offset;
		if (euler3param2 - euler3param[2] > 180)
			euler3param[2] = euler3param2 - 360;
		else if (euler3param2 - euler3param[2] < -180)
			euler3param[2] = euler3param2 + 360;
		else
			euler3param[2] = euler3param2;

		//if (i % 2 == 1) {
		for (int j = 0; j < 3; j++)
		{
			eulersum[j] += euler3param[j] * angleKp[j] + bodyangle[j];
		}
		rotationmatrix(rotmatrix3, -eulersum[0], eulersum[1], -eulersum[2]);
		//}

		// ステップごとの座標設定とzの補正
		posnow[0][0] = oldlegpos[0][0] + (x_a - oldlegpos[0][0]) * i / (step_times + 1);
		posnow[0][1] = oldlegpos[0][1] + (y_a - oldlegpos[0][1]) * i / (step_times + 1);
		posnow[0][2] = oldlegpos[0][2] + (z_a - oldlegpos[0][2]) * i / (step_times + 1);
		posnow[1][0] = oldlegpos[1][0] + (x_b - oldlegpos[1][0]) * i / (step_times + 1);
		posnow[1][1] = oldlegpos[1][1] + (y_b - oldlegpos[1][1]) * i / (step_times + 1);
		posnow[1][2] = oldlegpos[1][2] + (z_b - oldlegpos[1][2]) * i / (step_times + 1);
		posnow[2][0] = oldlegpos[2][0] + (x_c - oldlegpos[2][0]) * i / (step_times + 1);
		posnow[2][1] = oldlegpos[2][1] + (y_c - oldlegpos[2][1]) * i / (step_times + 1);
		posnow[2][2] = oldlegpos[2][2] + (z_c - oldlegpos[2][2]) * i / (step_times + 1);
		posnow[3][0] = oldlegpos[3][0] + (x_d - oldlegpos[3][0]) * i / (step_times + 1);
		posnow[3][1] = oldlegpos[3][1] + (y_d - oldlegpos[3][1]) * i / (step_times + 1);
		posnow[3][2] = oldlegpos[3][2] + (z_d - oldlegpos[3][2]) * i / (step_times + 1);
		all__height = all_height_old + (all_height - all_height_old) * i / (step_times + 1);

		// 座標変換
		if (attitude_control == 1)
		{
			rotatecoordinate(rotmatrix3, &posnow[0][0], &posnow[0][1], &posnow[0][2]);
			rotatecoordinate(rotmatrix3, &posnow[1][0], &posnow[1][1], &posnow[1][2]);
			rotatecoordinate(rotmatrix3, &posnow[2][0], &posnow[2][1], &posnow[2][2]);
			rotatecoordinate(rotmatrix3, &posnow[3][0], &posnow[3][1], &posnow[3][2]);
			//showmatrix(rotmatrix3);
		}

		// 本体の座標移動を記録
		//double delta[4][3];
		static double oldbodyheight = 0.0; // 前回の本体高さ
		if ((step_times + 1 - i) % deltastep == 0)
		{
			for (int j = 0; j < 4; j++)
			{
				for (int k = 0; k < 3; k++)
				{
					if (legflag[j] == 1)
						bodyposition[k] -= (posnow[j][k] - relativepos[j][k]) / num;
					relativepos[j][k] = posnow[j][k];
					// 脚の絶対座標を記録
					legposition[j][k] = bodyposition[k] + posnow[j][k];
					legposition[j][2] += all_height;
				}
			}
			bodyposition[2] += (all__height - oldbodyheight);
			oldbodyheight = all__height;
			//cout << int(bodyposition[0] * 100) << "	" << int(bodyposition[1] * 100) << "	" << int(bodyposition[2] * 100) << endl;
			if (redrawflag == 0)
				redrawflag = 1;
		}

		rf.x = posnow[0][0] - temp;
		rf.y = posnow[0][1] - temp;
		rb.x = posnow[1][0] - temp;
		rb.y = -posnow[1][1] - temp;
		lb.x = -posnow[2][0] - temp;
		lb.y = -posnow[2][1] - temp;
		lf.x = -posnow[3][0] - temp;
		lf.y = posnow[3][1] - temp;
#ifdef Z_ADJUST
		rf.z = posnow[0][2] + adjustz[0] * adjustrate[0];
		rb.z = posnow[1][2] + adjustz[1] * adjustrate[1];
		lb.z = posnow[2][2] + adjustz[2] * adjustrate[2];
		lf.z = posnow[3][2] + adjustz[3] * adjustrate[3];
#else
		rf.z = posnow[0][2];
		rb.z = posnow[1][2];
		lb.z = posnow[2][2];
		lf.z = posnow[3][2];
#endif

		rf.calc(rf.x, rf.y, rf.z, all__height, rf.theta0, rf.theta1, rf.theta2);
		rb.calc(rb.x, rb.y, rb.z, all__height, rb.theta0, rb.theta1, rb.theta2);
		lb.calc(lb.x, lb.y, lb.z, all__height, lb.theta0, lb.theta1, lb.theta2);
		lf.calc(lf.x, lf.y, lf.z, all__height, lf.theta0, lf.theta1, lf.theta2);

		Output_Angle(rf.theta0, rf.theta1, rf.theta2,
					 rb.theta0, rb.theta1, rb.theta2,
					 lb.theta0, lb.theta1, lb.theta2,
					 lf.theta0, lf.theta1, lf.theta2,
					 step_times, wait_time);

		wait(wait_time);
	}
	// 前回の足先座標の更新
	oldlegpos[0][0] = x_a;
	oldlegpos[0][1] = y_a;
	oldlegpos[0][2] = z_a;
	oldlegpos[1][0] = x_b;
	oldlegpos[1][1] = y_b;
	oldlegpos[1][2] = z_b;
	oldlegpos[2][0] = x_c;
	oldlegpos[2][1] = y_c;
	oldlegpos[2][2] = z_c;
	oldlegpos[3][0] = x_d;
	oldlegpos[3][1] = y_d;
	oldlegpos[3][2] = z_d;
	all_height_old = all_height;

	// 脚の絶対座標を記録
	/*for (int j = 0; j < 4; j++) {
		for (int k = 0; k < 3; k++) {
			legposition[j][k] = bodyposition[k] + posnow[j][k];
		}
		legposition[j][2] += all_height;
	}
	if (redrawflag == 0)redrawflag = 1;
	*/
	//oled.clear();
	//bno.get_angles();
	//pc.printf("(rf,rb,lb,lf)=(%lf,%lf,%lf,%lf)\n",rf.adjust,rb.adjust,lb.adjust,lf.adjust);
	//oled.printf("y=%2.4f", bodyposition[1]);
	//oled.printf("\n%2.1f %2.1f %2.1f", bno.euler.roll, bno.euler.pitch, bno.euler.yaw);

	/*****************************実質タイマー割り込み***************************/
	Get_gyro();
	OLED_display();

	/***************************************************************************/
#ifdef ODE
	wait(wait_time);
#else
	wait(wait_time / 10);
#endif
}
