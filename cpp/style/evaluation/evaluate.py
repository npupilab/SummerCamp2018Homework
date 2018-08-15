# -*- coding: utf-8 -*- 
import sys
import os

herepath=os.path.split(sys.argv[0])[0]
topic=sys.argv[1]
name=sys.argv[2]
scoreFile=sys.argv[3]
if herepath == '':
    herepath='.'
homework='{}/../{}/GSLAM/core'.format(herepath,name)
sys.path.append(herepath)

if not os.access(homework,os.R_OK):
    os.system('echo "[D]({}/evaluation/none.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

files=os.listdir(homework)
headerCount=0
filenames=[]
for file in files:
    if os.path.splitext(file)[1] == ".h":
        headerCount+=1
        filenames.append(os.path.join(homework,file))

if headerCount == 0:
    os.system('echo "[D]({}/evaluation/none.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

if headerCount < 3:
    os.system('echo "[C]({}/evaluation/less.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)
    

import cpplint as cl

#filenames = cl.ParseArguments(['{}/*.h'.format(homework)])
#cl.ParseArguments(['--root={}/../{}/include'.format(herepath,name),homework+"/*.h"])

cl._root='cpp/style/{}'.format(name)
backup_err = sys.stderr
try:
    sys.stderr = cl.codecs.StreamReader(sys.stderr,'replace')
    cl._cpplint_state.ResetErrorCounts()
    for filename in filenames:
      cl.ProcessFile(filename, cl._cpplint_state.verbose_level)
    cl._cpplint_state.PrintErrorCounts()
finally:
    sys.stderr = backup_err
    os.system('echo "[B]({}/evaluation/cpplint_failed.md)" > {}'.format(sys.argv[1],sys.argv[3]))

if cl._cpplint_state.error_count > 0:
    os.system('echo "[C,{}]({}/evaluation/error.md)" > {}'.format(cl._cpplint_state.error_count,sys.argv[1],sys.argv[3]))
    exit(0)

os.system('echo "[S]({}/{}/GSLAM/core)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
    

