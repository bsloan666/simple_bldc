import serial
import time

class ServoTest():
    def __init__(self):
        self.connection  = serial.Serial("/dev/cu.usbserial-410", 9600)
        self.connection.reset_input_buffer()

    def set_position(self, angle, speed):
        self.connection.write(f"10 {angle} {speed}\n".encode())

    def set_zero(self):
        self.connection.write(f"60\n".encode())

    def set_unlock(self):
        self.connection.write(f"40\n".encode())

    def step(self, direction, speed):
        self.connection.write(f"70 {direction} {speed}\n".encode())

    def calibrate(self):
        self.connection.write("80\n".encode())

    def dump_calibration(self):
        self.connection.write("90\n".encode())
        result = self.connection.read_until()
        return result.decode('utf-8')

    def get_position(self):
        self.connection.write(f"50\n".encode())
        result = self.connection.read_until()
        return result.decode('utf-8')


def run_quadrants(servo):
    print("Requsted {0}, Actual: {1}".format(0, servo.get_position().strip())) 

    stops = [316,  0]
    # stops = [76, 152, 228, 316, 228, 152, 76, 0]

    for position in stops:
        servo.set_position(position, 64)
        time.sleep(2)
        print("Requsted {0}, Actual: {1}".format(position, servo.get_position().strip())) 

    servo.set_position(-76, 128)
    time.sleep(2)
    print("Requsted {0}, Actual: {1}".format(-76, servo.get_position().strip())) 
    servo.set_unlock()
    # servo.connection.close()


def run_stepper(servo):
    for position in range(38):
        servo.step(-1, 100)
        time.sleep(0.08)
    for position in range(38):
        servo.step(1, 100)
        time.sleep(0.08)

    servo.set_unlock()

if __name__ == "__main__":
    servo = ServoTest()
  
    servo.set_zero()
    time.sleep(8)

    run_quadrants(servo)
    # servo.calibrate()
    # time.sleep(8)

    # text = servo.dump_calibration()

    # print(text)
    servo.set_unlock()
