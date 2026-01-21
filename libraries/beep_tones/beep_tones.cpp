#include "beep_tones.h"


BeepToneGenerator::BeepToneGenerator(){
}


void BeepToneGenerator::initialize(){
    /*
        Two octaves of the C-Major scale
    */
    tones[0] = 261.63 / 2;
    tones[1] = 293.66 / 2;
    tones[2] = 329.63 / 2;
    tones[3] = 349.23 / 2;
    tones[4] = 392.00 / 2;
    tones[5] = 440.00 / 2;
    tones[6] = 493.88 / 2;
 
    tones[7] = 261.63;
    tones[8] = 293.66;
    tones[9] = 329.63;
    tones[10] = 349.23;
    tones[11] = 392.00;
    tones[12] = 440.00;
    tones[13] = 493.88;

    tones[14] = 261.63 * 2;
    tones[15] = 293.66 * 2;
    tones[16] = 329.63 * 2;
    tones[17] = 349.23 * 2;
    tones[18] = 392.00 * 2;
    tones[19] = 440.00 * 2;
    tones[20] = 493.88 * 2;


    tones[21] = 261.63 * 4;
    tones[22] = 293.66 * 4;
    tones[23] = 329.63 * 4;

    Serial.begin(9600);
    Serial.flush();
    Serial.print("Beep tones initialized on pin 13");
    delay(100);
    Serial.end();
}   

void BeepToneGenerator::play(Note notes[], unsigned int count){
    int i;
    unsigned long accum;
    unsigned long musecs;
    noInterrupts();
    for(i = 0; i < count; i++){ 
        accum = 0;     
        musecs = 1000000 / notes[i].pitch * 0.5;
        while(accum < notes[i].length * 1000) {
            digitalWrite(13, HIGH);
            delayMicroseconds(musecs);
            digitalWrite(13, LOW);
            delayMicroseconds(musecs);
            accum += musecs;
        }
    }
    interrupts();
}

void BeepToneGenerator::test(){
    Note scale[48];
    int i;
    for(i = 0; i <24; i++){
        scale[i] = Note(tones[i], 64);
    }    
    for(i = 24; i <= 0; i--){
        scale[i] = Note(tones[i], 64);
    } 
    play(scale, 48);
}

