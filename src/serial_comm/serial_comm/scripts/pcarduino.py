#import serial
#import time

#ser = serial.Serial('/dev/ttyUSB1', 9600)  # replace 'COM_PORT' with your Arduino port

#def move_servo(angle):
#    ser.write(str(angle).encode())  # send the angle as a string encoded to bytes

#try:
#    while True:
#        angle = input("Enter angle (0 to 180): ")
#        move_servo(angle)
#        time.sleep(1)  # wait a bit for the servo to move
#except KeyboardInterrupt:
#    ser.close()  # close the serial connection when done


import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 9600)  # replace 'COM_PORT' with your Arduino port

def move_servo(angle):
    ser.write(str(angle).encode())  # send the angle as a string encoded to bytes

try:
    while True:
        # Read response from Arduino
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            print(f"Arduino says: {line}")
        
        # Get user input and send to Arduino
        angle = input("PC: Enter angle (0 to 180): ")
        move_servo(angle)
        time.sleep(1)  # wait a bit for the servo to move and Arduino to respond
except KeyboardInterrupt:
    print("Program exited by user")
    ser.close()  # close the serial connection when done

