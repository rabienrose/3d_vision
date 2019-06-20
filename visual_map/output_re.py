import os

d = '.'
with open(filepath,'w') as fw:
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
                            with open(filepath,'r') as fr:
                                fr.seek(0,0)
                                re_str = fr.readline()
                            fw.write(map_name+' + '+loc_name+' : '+re_str)
                        
            
