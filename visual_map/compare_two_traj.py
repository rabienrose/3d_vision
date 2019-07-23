import os
import sys
import math
f1_name = sys.argv[1]
f1_x_ind = int(sys.argv[2])
f1_y_ind = int(sys.argv[3])
f1_z_ind = int(sys.argv[4])
f2_name = sys.argv[5]
f2_x_ind = int(sys.argv[6])
f2_y_ind = int(sys.argv[7])
f2_z_ind = int(sys.argv[8])
f1 = open(f1_name, "r")
f2 = open(f2_name, "r")

line1 = f1.readline()
line2 = f2.readline()
total_error=0
total_count=0
while line1 or line2:
    line1 = f1.readline()
    line2 = f2.readline()
    line_splited1 = line1.split(',')
    line_splited2 = line2.split(',')
    if len(line_splited1)<=f1_z_ind:
        continue;
    if len(line_splited2)<=f2_z_ind:
        continue;
    d2x=(float(line_splited1[f1_x_ind])-float(line_splited2[f2_x_ind]))*(float(line_splited1[f1_x_ind])-float(line_splited2[f2_x_ind]))
    d2y=(float(line_splited1[f1_y_ind])-float(line_splited2[f2_y_ind]))*(float(line_splited1[f1_y_ind])-float(line_splited2[f2_y_ind]))
    d2z=(float(line_splited1[f1_z_ind])-float(line_splited2[f2_z_ind]))*(float(line_splited1[f1_z_ind])-float(line_splited2[f2_z_ind]))
    accu=float(line_splited1[f1_z_ind+1])
    if accu<1:
        error=math.sqrt(d2x+d2y)
        total_error=total_error+error
        total_count=total_count+1
print("avg err: "+str(total_error/total_count))

            
