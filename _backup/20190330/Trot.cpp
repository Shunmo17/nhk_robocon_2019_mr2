#include "Common.h"
#include "Output.h"

/**************************トロット歩容****************************/
void Trot(double x0, double y0, double all_height,
          double step_length, double step_height,
          double ratio_right, double ratio_left,
          int walk_times)
//aが進行方向左前。オフセットの方向の関係上、aはfrかblのみ。
//x0,y0：足先のxy座標
//step_length：歩幅
//step_height：足を上げたときの高さ
//ratio_right：右足の歩幅調整。0 < ratio_right < 1。
//ratio_left：左足の歩幅調整。0 < ratio_left < 1。
//walk_times：繰り返し回数。この回数をnとすると、2n + step_lengthだけ進むはず(実際には全然進まない)
{
    pc.printf("Start Trot!\n");

    /*Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      0,
                      0, 1); //xo、yoで足を浮かせた状態
    wait(0.10);*/

    /*Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      30, 0.05); //(1)*/

    //Wait_UB();

    for (int i(1); i <= walk_times; i++)
    {
        Output_Coordinate(x0, y0 - step_length * ratio_right, 0,
                          x0, y0 - step_length * ratio_right, 0,
                          x0, y0, 0,
                          x0, y0, 0,
                          all_height,
                          10, 0.03); //(2)

        Output_Coordinate(x0, y0 - (step_length * ratio_right / 2), step_height,
                          x0, y0 - (step_length * ratio_right / 2), 0,
                          x0, y0 - (step_length * ratio_left / 2), step_height,
                          x0, y0 - (step_length * ratio_left / 2), 0,
                          all_height,
                          10, 0.03); //(3)

        Output_Coordinate(x0, y0, 0,
                          x0, y0, 0,
                          x0, y0 - step_length * ratio_left, 0,
                          x0, y0 - step_length * ratio_left, 0,
                          all_height,
                          10, 0.03); //(4)

        Output_Coordinate(x0, y0 - (step_length * ratio_right / 2), 0,
                          x0, y0 - (step_length * ratio_right / 2), step_height,
                          x0, y0 - (step_length * ratio_left / 2), 0,
                          x0, y0 - (step_length * ratio_left / 2), step_height,
                          all_height,
                          10, 0.03); //(10)
    }

    Output_Coordinate(x0, y0 - step_length * ratio_right, 0,
                      x0, y0 - step_length * ratio_right, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      10, 0.03); //(2)

    Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      all_height,
                      10, 0.03); //(1)

    Output_Coordinate(x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      x0, y0, 0,
                      0,
                      30, 0.05); //xo、yoで足を浮かせた状態

    pc.printf("Finish Trot!\n");
}