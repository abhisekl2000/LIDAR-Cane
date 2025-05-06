#*****************************************************************************************
#Obstacle detection simplified: comparing X1 with X2 (opposite values of the right triangle)

#*****************************************************************************************
from tf_luna import TFLuna
import machine
from machine import Pin, RTC
import time
import sys
import _thread
from math import sin, cos, radians

# Global variables declarations
lidar1_height_to_floor = None
lidar2_diagonal_distance = None

# lidar = TFLuna()
#Parameters for an instance of lidar class: uartid, baudrate, tx_pin, rx_pin
lidar1 = TFLuna(1, 17, 16, 115200)
lidar2 = TFLuna(2, 19, 18, 115200)
buzzer = Pin(15, machine.Pin.OUT)

#Constants
fixed_angle = 45
theta_rad = radians(fixed_angle)
x_threshold = 0.3

# # Initialize RTC for startup flag check
# rtc = RTC()

# def check_first_boot():
#     # Try to read the stored boot flag from RTC memory
#     boot_flag = rtc.memory()
#     if not boot_flag:  # If the memory is empty, it's the first boot
#         print("First boot detected. Performing soft reset.")
#         # Perform soft reset (clear any state, etc.)
#         machine.soft_reset()
#         # Store the boot flag in RTC memory to prevent future resets
#         rtc.memory(b'boot_done')
#         return True
#     return False

# # Perform soft reset only once on first startup
# if check_first_boot():
#     time.sleep(2)  # Allow time for the reset process to take effect

#Startup
print("Executing the script!")

def power_noise():
    #3 beeps at the start of the device
    for i in range(3):
        buzzer.value(1)
        time.sleep(0.1)  # Beep duration
        buzzer.value(0)
        time.sleep(0.1)  # Interval between beeps

# Perform startup noise
power_noise()


def lidar_data_retrieval(lidar, thread_id):
    global lidar2_diagonal_distance, lidar1_height_to_floor
    try:
        while True:

            result = lidar.get_distance_strength_temperature()
            
            if result is None:
                print(f"Thread {thread_id} failed to get valid Lidar data") 
                buzzer.value(0)
                continue        #Skip the remaining code in current iteration
            
            #Process valid result
            distance, strength, temperature = result
            if thread_id == 1:
                lidar1_height_to_floor = distance
            elif thread_id == 2:
                lidar2_diagonal_distance = distance
            
            # Perform distance and obstacle checks only if both lidar readings are available
            if lidar1_height_to_floor and lidar2_diagonal_distance:
                print("Theta (radians):", theta_rad)
                X1 = lidar1_height_to_floor                     #since tan(45) = 1
                X2 = lidar2_diagonal_distance * sin(theta_rad)
                print(f"X1: {X1:.10f}, X2: {X2:.10f}")

                #Compare with x_threshold: if below threshold, then no obstacles
                if abs(X1 - X2) > x_threshold:
                    if X1 < X2:
                        print("Obstacle: structure detected")
                        
                    elif X2 > X1:
                        print("Obstacle: drop detected")

                    buzzer.value(1)     #Activate buzzer
                else:
                    buzzer.value(0)
            else:
                buzzer.value(0)         #Ensure buzzer is off if the distance is invalid


            print(f"Thread {thread_id} - Distance: {distance:.2f} m, Strength: {strength}, Temperature: {temperature:.2f} C")
            #print("Distance: {:.2f} m, Strength: {}, Temperature: {:.2f} C".format(distance, strength, temperature))
            time.sleep(0.1)

    except Exception as e:
        print("An error occured: ", e)

    finally:
        buzzer.value(0)


#Start threads for lidar1 and lidar2
_thread.start_new_thread(lidar_data_retrieval, (lidar1, 1))   #func: lidar_data_retrieval, args: tuple
_thread.start_new_thread(lidar_data_retrieval, (lidar2, 2))

print("Im here")

# Keep main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Program interrupted.")
