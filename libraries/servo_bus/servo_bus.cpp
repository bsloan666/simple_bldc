#include "servo_bus.h"
#include <Wire.h>

int command;
int data;

ServoBusMaster::ServoBusMaster(){

}

void ServoBusMaster::initialize(){
    Wire.begin();
}

void ServoBusMaster::send(int device, int command, int data){
    Wire.beginTransmission(device);
    Wire.write(command);
    Wire.write(data);
    Wire.endTransmission();
}

int ServoBusMaster::request(int device){
    Wire.requestFrom(device, 4);
    while (Wire.available()) {
        int response = Wire.read();
        return response;
    }
}

ServoBusSlave::ServoBusSlave(int dev_id):
    device_id(dev_id){
    command = 0;
    data = 0;
}

void ServoBusSlave::initialize(){
    Wire.begin(device_id);
    Wire.onRequest(respond_callback);
    Wire.onReceive(receive_callback);
}

void respond_callback(void){
    Wire.write(data);
}

void receive_callback(int quantity){
    while (Wire.available()) { 
        command = Wire.read();
        data = Wire.read();
    }
}

int ServoBusSlave::get_command(){
    return command;
}

int ServoBusSlave::get_data(){
    return data;
}
