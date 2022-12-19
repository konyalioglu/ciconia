__author__ = 'Geir Istad'
"""
MPU6050 Python I2C Class - MPU6050 example usage
Copyright (c) 2015 Geir Istad

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

from MPU6050 import MPU6050
import math
import time

i2c_bus = 1
device_address = 0x68
# The offsets are different for each device and should be changed
# accordingly using a calibration procedure
x_accel_offset = -83
y_accel_offset = 3934
z_accel_offset = -801
x_gyro_offset = -23
y_gyro_offset = 193
z_gyro_offset = 41
gravity = 9.81
enable_debug_output = True

mpu = MPU6050(i2c_bus, device_address, x_accel_offset, y_accel_offset,
              z_accel_offset, x_gyro_offset, y_gyro_offset, z_gyro_offset,
              enable_debug_output)

mpu.dmp_initialize()
mpu.set_DMP_enabled(True)
mpu_int_status = mpu.get_int_status()
print(hex(mpu_int_status))

packet_size = mpu.DMP_get_FIFO_packet_size()
print(packet_size)
FIFO_count = mpu.get_FIFO_count()
print(FIFO_count)
value = 0
count = 0
FIFO_buffer = [0]*64
counter = 0
FIFO_count_list = list()
calibre_a = 0.00123563121085
calibre_g = 3.65345386383e-08

print (' get ready for acceleration x1')
time.sleep(5)
print (' x1 started')
'''
while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        #print x1, y1, z1
        if abs(x1) > 1:
            continue
        else:
            value = x1 + value
            counter += 1
        if counter == 1000:
            valuex1 = value / counter
            

counter = 0
value = 0
print ' get ready for acceleration x2'
time.sleep(10)
print ' x2 started'

while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        if abs(x1) > 1:
            continue
        else:
            value = x1 + value
            #print x1, y1, z1
            counter += 1
        if counter == 1000:
            valuex2 = value / counter

offx = (valuex1 + valuex2) / 2
value = 0
counter = 0
print ' get ready for acceleration y1'
time.sleep(10)
print ' y1 started'
while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        if abs(y1) > 1:
            continue
        else:        
            value = value + y1
        #print x1, y1, z1
            counter += 1
        if counter == 1000:
            valuey1 = value / counter
value = 0            
print ' get ready for acceleration y2'           
counter = 0
time.sleep(10)
print ' y2 started'
while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        if abs(y1) > 1:
            continue
        else:        
            value = value + y1
            #print x1, y1, z1
            counter += 1
        if counter == 1000:
            valuey2 = value / counter
value = 0
counter = 0
print ' get ready for acceleration z1'
time.sleep(10)
offy = (valuey1 + valuey2) / 2
print ' z1 started'
while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        if abs(z1) > 1:
            continue
        else:        
            value = value + z1
            #print x1, y1, z1
            counter += 1
        if counter == 1000:
            valuez1 = value / counter

value = 0
print ' get ready for acceleration z2'
counter = 0
time.sleep(10)
print ' z2 started'
while counter < 1000:     
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()
          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
               
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(linear_accel.x)
        y1 = float(linear_accel.y)
        z1 = float(linear_accel.z)
        if abs(z1) > 1:
            continue
        else:
            value = value + z1
            #print x1, y1, z1
            counter += 1
        if counter == 1000:
            valuez2 = value / counter

offz = (valuez1 + valuez2) / 2
counter = 0
print offx, offy, offz
time.sleep(10)
'''
#while counter < 1000:
while True:
    FIFO_count = mpu.get_FIFO_count()
    mpu_int_status = mpu.get_int_status()
     
    # If overflow is detected by status or fifo count we want to reset
    if (FIFO_count == 1024) or (mpu_int_status & 0x10):
        mpu.reset_FIFO()

          
    # Check if fifo data is ready
    elif (mpu_int_status & 0x02):
        # Wait until packet_size number of bytes are ready for reading, default
        # is 42 bytes
        while FIFO_count < packet_size:
            FIFO_count = mpu.get_FIFO_count()
            
              
        FIFO_buffer = mpu.get_FIFO_bytes(packet_size)
        #accel = mpu.DMP_get_acceleration_int16(FIFO_buffer)
        quat = mpu.DMP_get_quaternion_int16(FIFO_buffer)
        grav = mpu.DMP_get_gravity(quat)
        euler_angles = mpu.DMP_get_euler_roll_pitch_yaw(quat, grav)
        accel = mpu.get_acceleration(FIFO_buffer, calibre_a)        
        grav = mpu.get_gravity(quat,calibre_g)
        linear_accel = mpu.linear_acceleration(accel, grav)
        x1 = float(euler_angles.x) #- offx
        y1 = float(euler_angles.y) #- offy
        z1 = float(euler_angles.z) #- offz
        print (x1, y1, z1)
        counter += 1
        mpu.reset_FIFO()
counter = 0
time.sleep(10)
print (offx, offy, offz)

        
