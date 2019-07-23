import os
import sys

file_name="/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/acuu_test/201907191.csv"
f1 = open(file_name, "r")
line1 = f1.readline()
x=[]
y=[]
z=[]
while line1:
    line_splited1 = line1.split(',')
    x.append(float(line_splited1[1]))
    y.append(float(line_splited1[2]))
    z.append(float(line_splited1[3]))
    line1 = f1.readline()
    
all_err_x=0
all_err_y=0
all_err_z=0
for i in range(0, 10):
    total_x=0
    total_y=0
    total_z=0
    temp_x=[]
    temp_y=[]
    temp_z=[]
    for j in range(0, 5):
        total_x=total_x+x[i+j*10]
        total_y=total_y+y[i+j*10]
        total_z=total_z+z[i+j*10]
        temp_x.append(x[i+j*10])
        temp_y.append(y[i+j*10])
        temp_z.append(z[i+j*10])
    avg_x=total_x/5
    avg_y=total_y/5
    avg_z=total_z/5
    total_err_x=0
    total_err_y=0
    total_err_z=0
    for j in range(0, 5):
        total_err_x=total_err_x+abs(temp_x[j]-avg_x)
        total_err_y=total_err_y+abs(temp_y[j]-avg_y)
        total_err_z=total_err_z+abs(temp_z[j]-avg_z)
    avg_err_x=total_err_x/5
    avg_err_y=total_err_y/5
    avg_err_z=total_err_z/5
    all_err_x=all_err_x+avg_err_x
    all_err_y=all_err_y+avg_err_y
    all_err_z=all_err_z+avg_err_z
    print(str(avg_err_x)+","+str(avg_err_y)+","+str(avg_err_z))
    
print("final err: ")
print(str(all_err_x/10)+","+str(all_err_y/10)+","+str(all_err_z/10))
    
    
