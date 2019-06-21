import sys
import yaml

assessname = '/raw_match.txt'
resultname = '/result_match.txt'
scalename = '/scale_match.txt'

def readymlconfig(filepath):
    content = {}
    with open(filepath,'r') as fr:
        fr.seek(0,0)
        yaml_sign = fr.readline()
        if (yaml_sign[0:6] == '%YAML:'):
            strlen = len(yaml_sign)
            fr.seek(strlen,0)
        data = fr.read()
        content = yaml.load(data)
    return content

def readassessfile(filepath):
    content = []
    with open(filepath,'r') as fr:
        lines = fr.readlines()
        for line in lines:
            par = line.split(',')
            linedist = (par[0],par[1],par[2])
            content.append(linedist)
    return content

def writeassessresult(filepath,result):
    with open(filepath,'w') as fw:
        fw.seek(0,0)
        re='';
        for line in result:
            re=re+str(line)+','
        fw.write(re)


if __name__ == "__main__":
    assesspath = sys.argv[1]
    configfile = sys.argv[2]
    assessfile = assesspath + assessname

    configinfo = readymlconfig(configfile)
    timecycle = configinfo["timecycle"]
    match_num_min = configinfo["match_num_min"]
    match_num_max = configinfo["match_num_max"]
    match_num_min_scale = configinfo["match_num_min_scale"]
    match_num_max_scale = configinfo["match_num_max_scale"]

    assessinfo = readassessfile(assessfile)
    istarttime = int(float(assessinfo[0][0]))
    iendtime = int(float(assessinfo[-1][0]))
    dur = float(iendtime - istarttime + 1)
    #print ("time dur :%f"%dur)

    #output result_match.txt
    cyclecount = 0.0
    mincount = 0
    midcount = 0
    maxcount = 0
    arrayRes = []
    total = len(assessinfo)
    index = -1
    all_match_num_min = []
    all_match_num_max = []
    total_exe_time=0
    while index < (total - 1):
        index += 1
        line = assessinfo[index]
        fline = int(float(line[0]))
        total_exe_time=total_exe_time+float(line[2])
        if fline < istarttime + timecycle:
            cyclecount += 1
            if int(line[1]) <= match_num_min:
                mincount += 1
            elif int(line[1]) >= match_num_max:
                maxcount += 1
            else:
                midcount += 1
            continue
        else:
            index -= 1
            arrayRes.append(str(istarttime)+','+str(mincount/cyclecount)+','+str(midcount/cyclecount)+','+str(maxcount/cyclecount)+'\n')
            all_match_num_min.append(mincount/cyclecount)
            all_match_num_max.append(maxcount/cyclecount)
            cyclecount = 0.0
            mincount = 0
            midcount = 0
            maxcount = 0
            istarttime += timecycle
    arrayRes.append(str(istarttime)+','+str(mincount/cyclecount)+','+str(midcount/cyclecount)+','+str(maxcount/cyclecount)+'\n')
    all_match_num_min.append(mincount/cyclecount)
    all_match_num_max.append(maxcount/cyclecount)
    resultfile = assesspath + resultname
    writeassessresult(resultfile,arrayRes)

    #output scale_match.txt
    greater_match_num_min = [b for b in all_match_num_min if b > match_num_min_scale]
    all_match_num_min_scale = 1-len(greater_match_num_min)/dur

    greater_match_num_max = [b for b in all_match_num_max if b > match_num_max_scale]
    all_match_num_max_scale = len(greater_match_num_max)/dur
    #print (len(greater_match_num_min)/dur)
    #print (len(greater_match_num_max)/dur)

    all_scale = []
    all_scale.append(all_match_num_min_scale)
    all_scale.append(all_match_num_max_scale)
    all_scale.append(total_exe_time/total)
    resultfile = assesspath + scalename
    writeassessresult(resultfile,all_scale)

    


