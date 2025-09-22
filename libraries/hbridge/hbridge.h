#ifndef __HBRIDGE_H__
#define __HBRIDGE_H__

#include <Ardiono.h>

class HBridge {
  public:
    HBridge(unsigned int _pin_a, unsigned int _pin_b);

    void initialize();
    void set(int velocity);

    

  private:
    unsigned int pin_a;
    unsigned int pin_b;
};

#endif
