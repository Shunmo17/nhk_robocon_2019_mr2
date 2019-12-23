#include "Common.h"
#include "Creeping.h"
#include "Output.h"

#define PI 3.14159265358979

double ratio_right(1.0), ratio_left(1.0); //ratio_right：右足の歩幅調整。0 < ratio_right < 1。ratio_left：左足の歩幅調整。0 < ratio_left < 1。
double initial_yaw;
bool slope_flag(0);

/************************プロトタイプ宣言*************************/
void Check_Yaw(double);
void Check_Yaw_Pcontrol(double);
//void Check_Sand(void);
//void Check_Slope(double);

/************************クリーピング歩容***********************************/

void Creeping(double x0, double y0, double y1, double all_height,
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
    {
        pc.printf("Start Creeping!\n");
    }

#ifdef CHANGE_OUTPUT
    initial_yaw = eulersum[2] - yaw_offset;
#else
    bno.get_angles();
    initial_yaw = bno.euler.yaw - yaw_offset;
#endif

    for (int i(1); i <= walk_times; i++)
    {
        if (i > 1)
        {
            yaw_ref = -60.0;
        }
        /*if (mode == 1)
        {
            Check_Sand();
        }

        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1, rb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(2)

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1, rf.adjust + step_height,
                          x0, -y1, rb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(3)-1

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        if (yaw_mode == 1)
        {
            //Check_Yaw(initial_yaw);
            Check_Yaw_Pcontrol(yaw_ref);
        }

        Output_Coordinate(x0, -y1 + step_length * ratio_right, rf.adjust + step_height,
                          x0, -y1, rb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(3)-2

        pc.printf("ratio_right= %lf\n", ratio_right);
        pc.printf("ratio_left= %lf\n", ratio_left);

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length * ratio_right, rf.adjust,
                          x0, -y1, rb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(4)

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        if (yaw_mode == 1)
        {
            //Check_Yaw(initial_yaw);
            Check_Yaw_Pcontrol(yaw_ref);
        }

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1 + step_length * ratio_left, lb.adjust,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4 * 2, creeping_wait_time * 2); //(5)

        /***********************************************************************/
        if (mode == 2 && slope_flag == 1 && abs(rb.adjust) <= 0.02) //坂を登り終えて1サイクル経ったらbreak
        {
            break;

            if (pc_debug == 3)
            {
                pc.printf("Finish climbing\n");
            }
        }
        /****************************************************************************/

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1 + step_length * ratio_left, lb.adjust + step_height,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(6)-1

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust + step_height,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(6)-2

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(7)

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1, lf.adjust + step_height,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(8)-1

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        if (yaw_mode == 1)
        {
            //Check_Yaw(initial_yaw);
            Check_Yaw_Pcontrol(yaw_ref);
        }

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1 + step_length * ratio_left, lf.adjust + step_height,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(8)-2

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1 + step_length / 2 * ratio_right, rf.adjust,
                          x0, -y1 + step_length / 2 * ratio_right, rb.adjust,
                          x0, -y1, lb.adjust,
                          x0, -y1 + step_length * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(9)

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        if (yaw_mode == 1)
        {
            //Check_Yaw(initial_yaw);
            Check_Yaw_Pcontrol(yaw_ref);
        }

        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1 + step_length * ratio_right, rb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4 * 2, creeping_wait_time * 2); //(10)

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1 + step_length * ratio_right, rb.adjust + step_height,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times, creeping_wait_time); //(11)-1

        /*if (mode == 1)
        {
            Check_Sand();
        }
        if (mode == 2)
        {
            Check_Slope(y0);
        }*/

        Output_Coordinate(x0, -y1, rf.adjust,
                          x0, -y1, rb.adjust + step_height,
                          x0, -y1 + step_length / 2 * ratio_left, lb.adjust,
                          x0, -y1 + step_length / 2 * ratio_left, lf.adjust,
                          all_height,
                          creeping_step_times * 4, creeping_wait_time); //(11)-2

        /***************************************************************/
        if (mode == 2 && slope_flag == 1 && abs(rb.adjust) <= 0.02) //坂を登り終えて1サイクル経ったらbreak
        {
            break;

            if (pc_debug == 3)
            {
                pc.printf("Finish climbing\n");
            }
        }
        /*******************************************************/
    }
    /*if (mode == 1)
    {
        Check_Sand();
    }
    if (mode == 2)
    {
        Check_Slope(y0);
    }*/

    Output_Coordinate(x0, -y1, rf.adjust,
                      x0, -y1, rb.adjust,
                      x0, -y1 + step_length / 2, lb.adjust,
                      x0, -y1 + step_length / 2, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(2)

    /*if (mode == 1)
    {
        Check_Sand();
    }
    if (mode == 2)
    {
        Check_Slope(y0);
    }*/

    Output_Coordinate(x0, -y1 + step_length / 2, rf.adjust,
                      x0, -y1 + step_length / 2, rb.adjust,
                      x0, -y1 + step_length / 2, lb.adjust,
                      x0, -y1 + step_length / 2, lf.adjust,
                      all_height,
                      creeping_step_times * 4, creeping_wait_time); //(1)

    if (pc_debug == 1)
    {
        pc.printf("Finish Creeping!\n");
    }
}

/************************進行方向確認**********************************/

void Check_Yaw(double initial_yaw)
{
    double alpha(10); //ズレの許容値[deg]
    double delta_yaw(0.0);

#ifdef CHANGE_OUTPUT
    yaw = eulersum[2] - yaw_offset;
#else
    bno.get_angles();
    yaw = bno.euler.yaw - yaw_offset;
#endif

    if (pc_debug == 1)
    {
        pc.printf("yaw= %lf\n", yaw);
    }

    delta_yaw = yaw - initial_yaw;

    if (delta_yaw > 100)
    {
        delta_yaw -= 360;
    }
    else if (delta_yaw < -100)
    {
        delta_yaw += 360;
    }

    if (-(3 / 2) * alpha < delta_yaw && delta_yaw < -(1 / 2) * alpha)
    {
        if (pc_debug == 1)
        {
            pc.printf("Litte Left\n");
        }
        ratio_right = 0.8;
        ratio_left = 1.0;
    }
    else if (-(5 / 2) * alpha < delta_yaw && delta_yaw < -(3 / 2) * alpha)
    {
        if (pc_debug == 1)
        {
            pc.printf("Left\n");
        }
        ratio_right = 0.7;
        ratio_left = 1.0;
    }
    else if (delta_yaw < -(5 / 2) * alpha)
    {
        if (pc_debug == 1)
        {
            pc.printf("Very Left\n");
        }
        ratio_right = 0.5;
        ratio_left = 1.0;
    }
    else if ((1 / 2) * alpha < delta_yaw && delta_yaw < (3 / 2) * alpha)
    {
        if (pc_debug == 1)
        {
            pc.printf("Litte Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.8;
    }
    else if ((3 / 2) * alpha < delta_yaw && delta_yaw < (5 / 2) * alpha)
    {
        if (pc_debug == 1)
        {
            pc.printf("Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.7;
    }
    else if ((5 / 2) * alpha < delta_yaw)
    {
        if (pc_debug == 1)
        {
            pc.printf("Very Right\n");
        }
        ratio_right = 1.0;
        ratio_left = 0.5;
    }
    else
    {
        if (pc_debug == 1)
        {
            pc.printf("Straight\n");
        }
        ratio_right = 1.0;
        ratio_left = 1.0;
    }
}

/************************Yaw角補正(P制御)******************************/
void Check_Yaw_Pcontrol(double _yaw_ref)
{
    double Kp_yaw(0.015);
    double delta_yaw(0.0);
    delta_yaw = _yaw_ref - yaw;
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
    if (ratio_right < 0.5)
    {
        ratio_right = 0.5;
    }
    if (ratio_left < 0.5)
    {
        ratio_left = 0.5;
    }
}

/*************************Sand確認************************************/
/*void Check_Sand(void)
{
    bno.get_angles();
    pitch = bno.euler.pitch - pitch_offset;
    roll = bno.euler.roll - roll_offset;

    if (pc_debug == 1)
    {
        pc.printf("pitch : %lf, roll : %lf\n", pitch, roll);
    }

    if (roll > 15)
    {
        if (pc_debug == 1)
        {
            pc.printf("front Up\n");
        }
        if (pitch > 0)
        {
            if (rf.adjust == 0.0)
            {
                rf.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z + rf.adjust,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (pc_debug == 1)
                {
                    pc.printf("rf adjust\n");
                }
            }
            else
            {
                if (pc_debug == 1)
                {
                    pc.printf("error\n");
                }
            }
        }
        else
        {
            if (lf.adjust == 0.0)
            {
                lf.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z + lf.adjust,
                                  all_height_old,
                                  3, 0.1);
                if (pc_debug == 1)
                {
                    pc.printf("lf adjust\n");
                }
            }
            else
            {
                if (pc_debug == 1)
                {
                    pc.printf("error\n");
                }
            }
        }
    }
    if (roll < -15)
    {
        pc.printf("back up\n");
        if (pitch > 0)
        {
            if (rb.adjust == 0.0)
            {
                rb.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z + rb.adjust,
                                  lb.x, lb.y, lb.z,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (pc_debug == 1)
                {
                    pc.printf("rb adjust\n");
                }
            }
            else
            {
                if (pc_debug == 1)
                {
                    pc.printf("error\n");
                }
            }
        }
        else
        {
            if (lb.adjust == 0.0)
            {
                lb.adjust = 0.12;
                Output_Coordinate(rf.x, rf.y, rf.z,
                                  rb.x, rb.y, rb.z,
                                  lb.x, lb.y, lb.z + lb.adjust,
                                  lf.x, lf.y, lf.z,
                                  all_height_old,
                                  3, 0.1);
                if (pc_debug == 1)
                {
                    pc.printf("lb adjust\n");
                }
            }
            else
            {
                if (pc_debug == 1)
                {
                    pc.printf("error\n");
                }
            }
        }
    }
    if (rf.adjust == 0.12 && rb.adjust == 0.12 && lf.adjust == 0.12 && lb.adjust == 0.12)
    {
        rf.adjust = rb.adjust = lf.adjust = lb.adjust = 0.0;
        Output_Coordinate(rf.x, rf.y, rf.z - 0.12,
                          rb.x, rb.y, rb.z - 0.12,
                          lb.x, lb.y, lb.z - 0.12,
                          lf.x, lf.y, lf.z - 0.12,
                          all_height_old,
                          3, 0.1);
        if (pc_debug == 1)
        {
            pc.printf("Reset all adjust!\n");
        }
    }
}
*/

/***************************Slope 確認*********************************/

/*
void Check_Slope(double y)
{
    bno.get_angles();
    roll = bno.euler.roll - roll_offset;
    rb.adjust = lb.adjust = -(2 * y + 0.195) * tan(roll * PI / 180);
    if (pc_debug == 1)
    {
        pc.printf("rb= %lf, lb= %lf\n", rb.adjust, lb.adjust);
    }
    if (abs(rb.adjust) >= 0.15) //坂を登り始めたフラグ
    {
        slope_flag = 1;
        if (pc_debug == 1)
        {
            pc.printf("slope_flag=1\n");
        }
    }
}
*/
