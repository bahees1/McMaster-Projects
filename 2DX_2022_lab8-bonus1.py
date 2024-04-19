#Edited and modified by Sarujan Baheerathan - 400453475 - bahees1

#Modify the following line with your own serial port details
#   Currently set COM3 as serial port at 115.2kbps 8N1
#   Refer to PySerial API for other options.  One option to consider is
#   the "timeout" - allowing the program to proceed if after a defined
#   timeout period.  The default = 0, which means wait forever.

import serial
import math
import numpy as np
import open3d as o3d

s = serial.Serial('COM3', 115200, timeout =10)

print("Opening: " + s.name)

# reset the buffers of the UART port to delete the remaining data in the buffers
s.reset_output_buffer()
s.reset_input_buffer()

# wait for user's signal to start the program
input("Press Enter to start communication...")
# send the character 's' to MCU via UART
# This will signal MCU to start the transmission
s.write('s'.encode())
# recieve 10 measurements from UART of MCU
f = open("tof_readings.xyz", "w")    #create a new file for writing 


for scan_count in range(3):
    
    for i in range(0,16):
        x = s.readline()

        if((i+1)*22.5 == 22.5):
        
            y_coord = int(float((math.sin(math.radians(22.5))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(22.5))))*float(x.decode()))
        
        elif((i+1)*22.5 == 45):
        
            y_coord = int(float((math.sin(math.radians(45))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(45))))*float(x.decode()))

        elif((i+1)*22.5 == 67.5):
        
            y_coord = int(float((math.sin(math.radians(67.5))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(67.5))))*float(x.decode()))

        elif((i+1)*22.5 == 90 ):
            y_coord = int(float(x.decode()))
            z_coord = 0

        elif((i+1)*22.5 == 112.5):
            y_coord = int(float((math.sin(math.radians(67.5))))*float(x.decode()))
            z_coord = int(float((math.cos(math.radians(67.5))))*float(x.decode()))
        
        elif((i+1)*22.5 == 135):
            y_coord = int(float((math.sin(math.radians(45))))*float(x.decode()))
            z_coord = int(float((math.cos(math.radians(45))))*float(x.decode()))

        elif((i+1)*22.5 == 157.5):
            y_coord = int(float((math.sin(math.radians(22.5))))*float(x.decode()))
            z_coord = int(float((math.cos(math.radians(22.5))))*float(x.decode()))
            
        elif ((i+1)*22.5 == 180):
            y_coord = 0
            z_coord = int(float(x.decode()))

        elif((i+1)*22.5 == 202.5):
            y_coord = int(-1 * float((math.sin(math.radians(22.5))))*float(x.decode()))
            z_coord = int(float((math.cos(math.radians(22.5))))*float(x.decode()))
            
        elif((i+1)*22.5 == 225):
            y_coord = int(-1 * float((math.sin(math.radians(45))))*float(x.decode()))
            z_coord = int(float((math.cos(math.radians(45))))*float(x.decode()))

        elif((i+1)*22.5 == 247.5):
            y_coord = int(-1 * float((math.sin(math.radians(67.5))))*float(x.decode()))
            z_coord= int(float((math.cos(math.radians(67.5))))*float(x.decode()))
            
        elif((i+1)*22.5 == 270 ):
            y_coord = int(-1 * float(x.decode()))
            z_coord = 0

        elif((i+1)*22.5 == 292.5):
            y_coord = int(-1 * float((math.sin(math.radians(67.5))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(67.5))))*float(x.decode()))
        
        elif((i+1)*22.5 == 315):
            y_coord = int(-1 * float((math.sin(math.radians(45))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(45))))*float(x.decode()))

        elif((i+1)*22.5 == 337.5):
            y_coord = int(-1 * float((math.sin(math.radians(22.5))))*float(x.decode()))
            z_coord = int(-1 * float((math.cos(math.radians(22.5))))*float(x.decode()))
            
        elif ((i+1)*22.5 == 360):
            y_coord = 0
            z_coord = int(-1*float(x.decode()))

        else:
            y_coord = 999
            z_coord = 999

            
        print("Y DATA BELOW")
        print(y_coord)
        print("Z DATA BELOW ")
        print(z_coord)
        print(" ")
        f.write('{0:d} {1:d} {2:d}\n'.format((p+1)*100, y_coord, z_coord))    #write x,0,0 (xyz) to file as p1
        
f.close()   #there should now be a file containing 40 vertex coordinates

#Read the test data in from the file we created        
print("Read in the prism point cloud data (pcd)")
pcd = o3d.io.read_point_cloud("tof_readings.xyz", format="xyz")

#Lets see what our point cloud data looks like numerically       
print("The PCD array:")
print(np.asarray(pcd.points))

#Lets see what our point cloud data looks like graphically       
print("Lets visualize the PCD: (spawns seperate interactive window)")
o3d.visualization.draw_geometries([pcd])

#Give each vertex a unique number
yz_slice_vertex = []
for x in range(0,48):
    yz_slice_vertex.append([x])

#Define coordinates to connect lines in each yz slice        
lines = []

for h in range(0,3):
    
    for y in range(16*h,(16*(h+1))-1):
        lines.append([yz_slice_vertex[y], yz_slice_vertex[y+1]])

    if(h != 2):
        lines.append([yz_slice_vertex[(16*(h+1))-1], yz_slice_vertex[16*h]])
    
        
    #Define coordinates to connect lines between current and next yz slice        
for x in range(0,32):
    lines.append([yz_slice_vertex[x], yz_slice_vertex[x+16]])
       

#This line maps the lines to the 3d coordinate vertices
line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))

#Lets see what our point cloud data with lines looks like graphically       
o3d.visualization.draw_geometries([line_set])
    
#close the port
print("Closing: " + s.name)
s.close()






