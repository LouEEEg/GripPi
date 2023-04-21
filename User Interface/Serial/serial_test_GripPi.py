import serial

def GripPiSerial(GripPiTx):
    GripPiRx=77
    
    if __name__ == '__main__':
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.reset_input_buffer()
        ser.write(GripPiTx)
        #while GripPiRx == 77:
         #   while ser.in_waiting > 0:
          #      GripPiRx = ser.read_until().decode('utf-8').rstrip()
    return GripPiRx       

home = "<800,800,0,90>"
safe = "<300,300,315,90>"

bin1_pos = "<485,300,0,90>"
bin2_pos = "<505,300,150,90>" #505 results may vary
bin3_pos = "<485,300,315,90>"

camera_pos = "<300,100,340,90>"
camera_grip = "<300,100,340,130>"

user_pos = "<125,100,310,90>"

x_b = bytes(home,'utf-8')
test = GripPiSerial(x_b)
print(test)
