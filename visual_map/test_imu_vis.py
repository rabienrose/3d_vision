import matplotlib.pyplot as plt

f = open("/media/chamo/095d3ecf-bef8-469d-86a3-fe170aec49db/test_imu/chamo.txt", "r")
content = f.readlines()
file_name=[]
time_stamp=[]
acce=[]
gyro=[]
scale_conf=[]
grav_conf=[]
scale=[]

for line in content:
    line_splited = line.split(',')
    file_name.append(line_splited[0])
    time_stamp.append(float(line_splited[1]))
    acce.append(float(line_splited[2]))
    gyro.append(float(line_splited[3]))
    scale_conf.append(float(line_splited[4])*1000)
    grav_conf.append(float(line_splited[5]))
    scale.append(float(line_splited[6])/100)

plt.plot(scale_conf) 
plt.plot(scale)  
plt.show()
