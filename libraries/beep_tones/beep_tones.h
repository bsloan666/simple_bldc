#ifndef __BEEP_TONES_H__
#define __BEEP_TONES_H__

#include <Arduino.h>

class Note {
    public:
        float pitch;
        unsigned long length;
        Note(): pitch(261.63), length(64){} 
        Note(float _p, unsigned long _l): pitch(_p), length(_l){} 
};

class BeepToneGenerator {
    public:
        BeepToneGenerator();
        void initialize();
        void play(Note notes[], unsigned int count); 
        void test();
        float tones[24];
};



#endif
