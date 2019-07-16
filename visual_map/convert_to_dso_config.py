import matplotlib.pyplot as plt
import sys
if __name__ == "__main__":
    config_file_addr = sys.argv[1]
    f_cam = open(config_file_addr+"/camera_config.txt", "r")
    line_cam = f_cam.readline()
    line_splited = line_cam.split(',')
    
    fx=float(line_splited[0])
    fy=float(line_splited[1])
    cx=float(line_splited[2])
    cy=float(line_splited[3])
    k1=float(line_splited[4])
    k2=float(line_splited[5])
    p1=float(line_splited[6])
    p2=float(line_splited[7])
    f_image = open(config_file_addr+"/image_conf.txt", "r")
    line_img = f_image.readline()
    line_splited = line_img.split(',')
    width=int(line_splited[0])
    height=int(line_splited[1])
    f_write = open(config_file_addr+"/camera.txt", "w")
    line1 = str(fx)+" "+str(fy)+" "+str(cx)+" "+str(cy)+" "+str(k1)+" "+str(k2)+" "+str(p1)+" "+str(p2)
    line2 = str(width)+" "+str(height)
    line3 = str(fx/width)+" "+str(fy/height)+" "+str(cx/width)+" "+str(cy/height)+" 0"
    line4 = str(width)+" "+str(height)
    f_write.write(line1+"\n")
    f_write.write(line2+"\n")
    f_write.write(line3+"\n")
    f_write.write(line4+"\n")
