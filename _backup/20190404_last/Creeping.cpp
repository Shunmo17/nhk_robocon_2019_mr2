#include "Common.h"
#include "Creeping.h"
#include "Output.h"
#include "Function.h"

#define PI 3.14159265358979
#define MAX_RATIO 0.4

double ratio_right(1.0), ratio_left(1.0); //ratio_right：右足の歩幅調整。0 < ratio_right < 1。ratio_left：左足の歩幅調整。0 < ratio_left < 1。
double initial_yaw;

/************************プロトタイプ宣言*************************/
void Check_Yaw(double);
void Check_x(double);
/************************クリーピング歩容***********************************/

void Creeping(double x0, double y0, double y1, double all_height, double adjust_y,
              double step_length, double step_height,
              double creeping_step_times, double creeping_wait_time,
              int walk_times)
//x0,y0：足先のxy座標
//y1：3足立ちのときのy座標
//step_length：歩幅
//step_height：足を上げたときの高さ
//curve：はじめ向いている方向と進行方向のなす角度[deg]
//walk_times：繰り返し回数。この回数をnとすると、2n + step_lengthだけ進むはず(実際には全然進まない)
{

    if (pc_debug == 1)
        pc.printf("Start Creeping!\n");

#ifdef CHANGE_OUTPUT
    initial_yaw = eulersum[2] - yaw_offset;
#else
    bno.get_angles();
    initial_yaw = bno.euler.yaw - yaw_offset;
#endif

    for (int i(1); i <= walk_times; i++)
    {
        Creeping_repeat_2step(x0, y0, y1, all_height, adjust_y,
                              step_length, step_height,
                              creeping_step_times, creeping_wait_time);
    }

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(2)

    if (pc_debug == 1)
        pc.printf("Finish Creeping!\n");
}

/*************************Creepingの繰り返すところ(2歩)******************/
void Creeping_repeat_2step(double x0, double y0, double y1, double all_height, double adjust_y,
                           double step_length, double step_height,
                           double creeping_step_times, double creeping_wait_time)
{
    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(2)

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust + step_height,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(3)-1

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y + step_length * ratio_right, rf.adjust + step_height,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(3)-2

    Output_Coordinate(x0, -y1 - adjust_y + step_length * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(4)

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4 * 2, creeping_wait_time * 2); //(5)

    Position_estimate(step_length);

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_left, lb.adjust + step_height,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(6)-1

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust + step_height,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(6)-2

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(7)

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust,
                      x0, -y1 - adjust_y, lf.adjust + step_height,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(8)-1

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust,
                      x0, -y1 - adjust_y + step_length * ratio_left, lf.adjust + step_height,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(8)-2

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust,
                      x0, -y1 - adjust_y + step_length * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(9)

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4 * 2, creeping_wait_time * 2); //(10)

    Position_estimate(step_length);

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_right, rb.adjust + step_height,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(11)-1

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust + step_height,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(11)-2
}

/***********************Creepingの繰り返すところ(1歩)*******************/
void Creeping_repeat_1step(double x0, double y0, double y1, double all_height, double adjust_y,
                           double step_length, double step_height,
                           double creeping_step_times, double creeping_wait_time)
{
    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(2)

    Output_Coordinate(x0, -y1 - adjust_y, rf.adjust + step_height,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(3)-1

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y + step_length * ratio_right, rf.adjust + step_height,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(3)-2

    Output_Coordinate(x0, -y1 - adjust_y + step_length * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y, rb.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y + step_length / 2 * ratio_left, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(4)

    if (yaw_mode == 1)
        Check_Yaw(yaw_ref);

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_left, lb.adjust,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4 * 2, creeping_wait_time * 2); //(5)

    Position_estimate(step_length);

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y + step_length * ratio_left, lb.adjust + step_height,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times, creeping_wait_time); //(6)-1

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust + step_height,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(6)-2

    adjust_y = 0.0;

    Output_Coordinate(x0, -y1 - adjust_y + step_length / 2 * ratio_right, rf.adjust,
                      x0, -y1 + adjust_y + step_length / 2 * ratio_right, rb.adjust,
                      x0, -y1 + adjust_y, lb.adjust,
                      x0, -y1 - adjust_y, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(7)
}

/***********************leg_type=0から大の字************************/
void LegType0_to_Dainozi()
{
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.10;
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.0;
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
}

/***********************leg_type=1から大の字************************/
void LegType1_to_Dainozi()
{
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1, lb.adjust + STEP_HEIGHT,
                      X0, -Y1, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust + STEP_HEIGHT,
                      X0, -Y1, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.10;
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1, lf.adjust + STEP_HEIGHT,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust + STEP_HEIGHT,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.0;
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
}

/***********************大の字からleg_type=0************************/
void Dainozi_to_LegType0()
{
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.10;
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    Output_Coordinate(X0, -Y1, rf.adjust,
                      X0, -Y1, rb.adjust + STEP_HEIGHT,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
    adjust_y = 0.0;
    Output_Coordinate(X0, -Y1 + STEP_LENGTH / 2, rf.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, rb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lb.adjust,
                      X0, -Y1 + STEP_LENGTH / 2, lf.adjust,
                      ALL_HEIGHT,
                      CREEPING_STEP_TIMES, CREEPING_WAIT_TIME);
}

/************************Yaw角補正(P制御)******************************/
void Check_Yaw(double _yaw_ref)
{
    double Kp_yaw(0.06);
    double delta_yaw(0.0);
    /*
    if (abs(_yaw_ref - yaw) < abs(_yaw_ref - yaw + 360))
    {
        if (abs(_yaw_ref - yaw) < abs(_yaw_ref - yaw - 360))
            delta_yaw = _yaw_ref - yaw;
        else
            delta_yaw = _yaw_ref - yaw - 360;
    }
    else if (abs(_yaw_ref - yaw + 360) < abs(_yaw_ref - yaw))
    {
        if (abs(_yaw_ref - yaw + 360) < abs(_yaw_ref - yaw - 360))
            delta_yaw = _yaw_ref - yaw + 360;
        else
            delta_yaw = _yaw_ref - yaw - 360;
    }*/
    if (delta_yaw >= 0)
    {
        ratio_left = 1.0;
        ratio_right = 1.0 - Kp_yaw * delta_yaw;
    }
    else
    {
        ratio_right = 1.0;
        ratio_left = 1.0 - Kp_yaw * abs(delta_yaw);
    }
    if (ratio_right < 0.4)
        ratio_right = 0.4;
    if (ratio_left < 0.4)
        ratio_left = 0.4;
}

/****************************x軸補正***********************/
void Check_x(double x_ref)
{
    double Kp_x(0.1);
    double delta_x(0.0);
    delta_x = x_now - x_ref;
    if (delta_x >= 0)
    {
        ratio_right = 1.0;
        ratio_left = 1.0 - Kp_x * abs(delta_x);
    }
    else
    {
        ratio_left = 1.0;
        ratio_right = 1.0 - Kp_x * delta_x;
    }
    if (ratio_right < 0.4)
        ratio_right = 0.4;
    if (ratio_left < 0.4)
        ratio_left = 0.4;
}