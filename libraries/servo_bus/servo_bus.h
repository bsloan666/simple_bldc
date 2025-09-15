#ifndef __SERVO_BUS_H__
#define __SERVO_BUS_H__

#include <Arduino.h>

class ServoBusMaster{
    public:
        ServoBusMaster();

        void initialize();

        void send(int device, int command, int position);

        int request(int device);
};


class ServoBusSlave{
    public:
        ServoBusSlave(int dev_id);

        void initialize(void);

        int get_command(); 
        int get_data(); 

    private:
        int device_id;
        
};

void respond_callback(void);
void receive_callback(int quantity);

#endif
