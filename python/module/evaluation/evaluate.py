# -*- coding: utf-8 -*- 
import os
import sys
import importlib
import random

#print("args is ",sys.argv);# ["evaluate.py", "python/hellopython", "赵勇","python/hellopython/赵勇/Score.txt"] 
#sys.argv=["",".","zy","score.txt"]

herepath=os.path.split(sys.argv[0])[0]
topicFolder=os.path.join(herepath,"..")
homeworkFolder='{}/{}'.format(topicFolder,sys.argv[2])

if len(sys.argv)<4:
    exit(0)

if not os.access(homeworkFolder,os.R_OK):
    os.system('echo "[D]({}/evaluation/none_init.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

has_package=0
dirs=os.listdir(homeworkFolder)
for d in dirs:
    if os.access('{}/{}/__init__.py'.format(homeworkFolder,d),os.R_OK):
       has_package=1

if (not has_package):
    os.system('echo "[D]({}/evaluation/none_init.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

if not os.access('{}/test_package.py'.format(homeworkFolder),os.R_OK):
    os.system('echo "[D]({}/evaluation/none_init.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)


#os.system('echo "[S]({}/{}/test_package.py)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
#exit(0)

os.system('rm -f plot.png data.txt'.format(homeworkFolder))
os.system('cp -p {}/data.txt .'.format(homeworkFolder))
sys.path.append(homeworkFolder)

fd = os.popen('python {}/{}/test_package.py'.format(topicFolder,sys.argv[2]))
result = fd.read()
fd.close()

if not os.access('plot.png',os.R_OK):
    os.system('echo "[C]({}/evaluation/none_plot.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

if result.find("PASS")>=0 :
   os.system('echo "[S]({}/{}/test_package.py)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
else:
   print (result)
   os.system('echo "[B]({}/evaluation/wrong_output.md)" > {}'.format(sys.argv[1],sys.argv[3]))







