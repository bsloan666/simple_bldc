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
        ServoBusSlave();

        void initialize(dev_id);

        int get_command(); 
        int get_data(); 
        void set_data(int new_data); 

    private:
        int device_id;
        
};

void respond_callback(void);
void receive_callback(int quantity);

#endif
