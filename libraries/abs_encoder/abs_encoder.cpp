#include "abs_encoder.h"


int gray_lookup[256] = {
     0,   1,   3,   2,   7,   6,   4,   5,
    15,  14,  12,  13,   8,   9,  11,  10,
    31,  30,  28,  29,  24,  25,  27,  26,
    16,  17,  19,  18,  23,  22,  20,  21,
    63,  62,  60,  61,  56,  57,  59,  58,
    48,  49,  51,  50,  55,  54,  52,  53,
    32,  33,  35,  34,  39,  38,  36,  37,
    47,  46,  44,  45,  40,  41,  43,  42,
   127, 126, 124, 125, 120, 121, 123, 122,
   112, 113, 115, 114, 119, 118, 116, 117,
    96,  97,  99,  98, 103, 102, 100, 101,
   111, 110, 108, 109, 104, 105, 107, 106,
    64,  65,  67,  66,  71,  70,  68,  69,
    79,  78,  76,  77,  72,  73,  75,  74,
    95,  94,  92,  93,  88,  89,  91,  90,
    80,  81,  83,  82,  87,  86,  84,  85,
   255, 254, 252, 253, 248, 249, 251, 250,
   240, 241, 243, 242, 247, 246, 244, 245,
   224, 225, 227, 226, 231, 230, 228, 229,
   239, 238, 236, 237, 232, 233, 235, 234,
   192, 193, 195, 194, 199, 198, 196, 197,
   207, 206, 204, 205, 200, 201, 203, 202,
   223, 222, 220, 221, 216, 217, 219, 218,
   208, 209, 211, 210, 215, 214, 212, 213,
   128, 129, 131, 130, 135, 134, 132, 133,
   143, 142, 140, 141, 136, 137, 139, 138,
   159, 158, 156, 157, 152, 153, 155, 154,
   144, 145, 147, 146, 151, 150, 148, 149,
   191, 190, 188, 189, 184, 185, 187, 186,
   176, 177, 179, 178, 183, 182, 180, 181,
   160, 161, 163, 162, 167, 166, 164, 165,
   175, 174, 172, 173, 168, 169, 171, 170,
  
};


AbsoluteRotaryEncoder::AbsoluteRotaryEncoder():
{
   
}

void AbsoluteRotaryEncoder::initialize(){

    // analog pin assignments
    pins[0] = A0;
    pins[1] = A1;
    pins[2] = A2;
    pins[3] = A3;
    
    // skipping A4 and A5 as they're used for I2C comms
    pins[4] = A6;
    pins[5] = A7;

    // hijacking 2 digital pins for the end bits
    pins[6] = 11;
    pins[7] = 12;

    
    pinMode(11, INPUT);
    pinMode(12, INPUT);

    Serial.begin(9600);

    Serial.print("AbsoluteRotaryEncoder running");

    delay(100);
    Serial.end();
}

int AbsoluteRotaryEncoder::read(){
    int val8 = digitalRead(pins[0]) == HIGH? 1: 0;
    int val7 = digitalRead(pins[1]) == HIGH? 1: 0;
    int val6 = digitalRead(pins[2]) == HIGH? 1: 0;
    int val5 = digitalRead(pins[3]) == HIGH? 1: 0;

    // Special case for pins A6 and A7, which are analog-only
    int val4 = analogRead(pins[4]) > 500? 1: 0;
    int val3 = analogRead(pins[5]) > 500? 1: 0;

    int val2 = digitalRead(pins[6]) == HIGH? 1: 0;
    int val1 = digitalRead(pins[7]) == HIGH? 1: 0;

    // DEBUG
    Serial.begin(9600);

    Serial.print(" ");
    Serial.print(val8);
    Serial.print(" ");
    Serial.print(val7);
    Serial.print(" ");
    Serial.print(val6);
    Serial.print(" ");
    Serial.print(val5);
    Serial.print(" ");
    Serial.print(val4);
    Serial.print(" ");
    Serial.print(val3);
    Serial.print(" ");
    Serial.print(val2);
    Serial.print(" ");
    Serial.println(val1);

    delay(1000);
    Serial.end();
    // DEBUG


    long graycode = val1 << 7 | val2 << 6 | val3 << 5 | val4 << 4 | val5 << 3 | val6 << 2 | val7 << 1 | val8;
    long pos = gray_lookup[graycode];

    return pos;
}
