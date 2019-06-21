import os
import sys
d = sys.argv[1]
out_file=os.path.join(d, 'final_reuslt.txt')
with open(out_file,'w') as fw:
    fw.seek(0,0)
    file_list = os.listdir(d)
    for folder in file_list:
        full_addr = os.path.join(d, folder)
        if os.path.isdir(full_addr):
            if ".bag" in folder:
                map_name=folder
                file_list1 = os.listdir(full_addr)
                for folder1 in file_list1:
                    full_addr1 = os.path.join(full_addr, folder1)
                    if os.path.isdir(full_addr1):
                        if ".bag" in folder1:
                            loc_name=folder1
                            print(full_addr1)
                            if not os.path.isfile(os.path.join(full_addr1, 'raw_match.txt')): 
                                fw.write(map_name+' + '+loc_name+' : '+'failed!!!\n')
                                continue
                            os.system('python ./assessLoc.py '+full_addr1+' ./assessConfig.yaml')
                            with open(os.path.join(full_addr1, 'scale_match.txt'),'r') as fr:
                                fr.seek(0,0)
                                re_str = fr.readline()
                                fw.write(map_name+' + '+loc_name+' : '+re_str+'\n')
                        
            
