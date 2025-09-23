#include "servo_bus.h"
#include <Wire.h>

int command;
int data;
String receivedString = "";
char in_buffer[32];
char out_buffer[32];

ServoBusMaster::ServoBusMaster(){

}

void ServoBusMaster::initialize(){
    Wire.begin();
    Serial.begin(9600);

    Serial.println("ServoBusMaster running");

    delay(100);
    Serial.end();
}

void ServoBusMaster::send(int device, int new_command, int new_data){
    sprintf(out_buffer, "%d %d", new_command, new_data);
    Wire.beginTransmission(device);
    Wire.write(out_buffer);
    Wire.endTransmission();
}

int ServoBusMaster::request(int device){
    Wire.requestFrom(device, 8);
    int i = 0;
    while (Wire.available() && i < sizeof(in_buffer) - 1) {
        in_buffer[i++] = Wire.read();
    }
    sscanf(in_buffer, "%d", &data);
    return data;
}

ServoBusSlave::ServoBusSlave(){
    command = 0;
    data = 0;
}

void ServoBusSlave::initialize(int dev_id){
    device_id = dev_id;
    Wire.begin(device_id);
    Wire.onRequest(respond_callback);
    Wire.onReceive(receive_callback);
    Serial.begin(9600);

    Serial.print("ServoBusSlave running at address ");
    Serial.println(device_id);

    delay(100);
    Serial.end();
}

void respond_callback(void){
    sprintf(out_buffer, "%d", data);
    Wire.write(out_buffer);
}

void receive_callback(int quantity){
    int i = 0;
    while (Wire.available() && i < sizeof(in_buffer) - 1) {
        in_buffer[i++] = Wire.read();
    }
    in_buffer[i] = '\0';
    sscanf(in_buffer, "%d %d", &command, &data);
}

int ServoBusSlave::get_command(){
    return command;
}

void ServoBusSlave::reset_command(){
    command = 0;
}

int ServoBusSlave::get_data(){
    return data;
}

void ServoBusSlave::set_data(int new_data){
    data = new_data;
}
