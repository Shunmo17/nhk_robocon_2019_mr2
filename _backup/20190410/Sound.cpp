#include <mbed.h>
#include "Common.h"

#define sound_size 8

#define mC 261.626
#define mD 293.665
#define mE 329.628
#define mF 349.228
#define mG 391.995
#define mA 440.000
#define mB 493.883

/*void Startup_Sound()
{
    double startup_sound[sound_size] = {mC, mD, mE, mF, mG, mA, mB, mC * 2};
    for (int i(0); i < sound_size; i++)
    {
        speaker.period(1.0 / startup_sound[i]);
        speaker.write(0.5f);
        wait(0.1f);
        speaker.write(0.0f);
    }
}*/