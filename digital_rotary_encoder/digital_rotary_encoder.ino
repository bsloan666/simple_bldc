#include "TM1637Display.h"

/*
  sample a digital rotary encoder with 64 positions. 
*/

long prev_pos;
long prev_prev_pos;

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


int degree_lookup[256] = {
     0,   1,   3,   4,   6,   7,   8,  10,
    11,  13,  14,  16,  17,  18,  20,  21,
    23,  24,  25,  27,  28,  30,  31,  32,
    34,  35,  37,  38,  40,  41,  42,  44,
    45,  47,  48,  49,  51,  52,  54,  55,
    56,  58,  59,  61,  62,  64,  65,  66,
    68,  69,  71,  72,  73,  75,  76,  78,
    79,  80,  82,  83,  85,  86,  88,  89,
    90,  92,  93,  95,  96,  97,  99, 100,
   102, 103, 104, 106, 107, 109, 110, 112,
   113, 114, 116, 117, 119, 120, 121, 123,
   124, 126, 127, 128, 130, 131, 133, 135,
   136, 137, 138, 140, 141, 143, 144, 145,
   147, 148, 150, 151, 152, 154, 155, 157,
   158, 160, 161, 162, 164, 165, 167, 168,
   169, 171, 172, 174, 175, 176, 177, 178,
   189, 180, 184, 185, 186, 188, 189, 191,
   192, 193, 195, 196, 198, 199, 200, 202,
   203, 205, 206, 208, 209, 210, 212, 213,
   215, 216, 217, 219, 220, 222, 223, 224,
   226, 227, 229, 230, 232, 233, 234, 236,
   237, 239, 240, 241, 243, 244, 246, 247,
   248, 250, 251, 253, 254, 256, 257, 258,
   260, 261, 263, 264, 265, 267, 268, 270,
   271, 272, 274, 275, 277, 278, 280, 281,
   282, 284, 285, 287, 288, 289, 291, 292,
   294, 295, 296, 298, 299, 301, 302, 304,
   305, 306, 308, 309, 311, 312, 313, 315,
   316, 318, 319, 320, 322, 323, 325, 326,
   328, 329, 330, 332, 333, 335, 336, 337,
   339, 340, 342, 343, 344, 346, 347, 349,
   350, 352, 353, 354, 356, 357, 359, 360
};


TM1637Display display = TM1637Display(11, 10);
// the setup function runs once when you press reset or power the board

void beep() {
  int i;
  for (i = 0; i < 30; i++) {
    digitalWrite(2, HIGH);
    delayMicroseconds(500);
    digitalWrite(2, LOW);
    delayMicroseconds(500);
  }
}

void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  display.clear();
  pinMode(2, OUTPUT);
  display.setBrightness(7);
  // Serial.begin(9600);
}

// the loop function runs over and over again forever
void loop() {
if(1){
  int val8 = analogRead(A0) > 500? 1: 0;
  int val7 = analogRead(A1) > 500? 1: 0;
  int val6 = analogRead(A2) > 500? 1: 0;
  int val5 = analogRead(A3) > 500? 1: 0;
  int val4 = analogRead(A4) > 500? 1: 0;
  int val3 = analogRead(A5) > 500? 1: 0;
  int val2 = analogRead(A6) > 500? 1: 0;
  int val1 = analogRead(A7) > 500? 1: 0;
  
  long graycode = val1 << 7 | val2 << 6 | val3 << 5 | val4 << 4 | val5 << 3 | val6 << 2 | val7 << 1 | val8;
  long pos = gray_lookup[graycode];


  if(prev_pos != pos){
    display.showNumberDecEx(pos, 0, false, 4, 0);
    // Serial.println(pos);
    beep();
    delay(50);
  }
  prev_prev_pos = prev_pos;
  prev_pos = pos;

} else {
  int val8 = 1000 + analogRead(A0);
  int val7 = 1000 + analogRead(A1);
  int val6 = 1000 + analogRead(A2);
  int val5 = 1000 + analogRead(A3);
  int val4 = 1000 + analogRead(A4);
  int val3 = 1000 + analogRead(A5);
  int val2 = 1000 + analogRead(A6);
  int val1 = 1000 + analogRead(A7);
  
  Serial.print(val1);
  Serial.print(" ");
  Serial.print(val2);
  Serial.print(" ");
  Serial.print(val3);
  Serial.print(" ");
  Serial.print(val4);
  Serial.print(" ");
  Serial.print(val5);
  Serial.print(" ");
  Serial.print(val6);
  Serial.print(" ");
  Serial.print(val7);
  Serial.print(" ");
  Serial.print(val8);
  Serial.println(" ");
} 


}
