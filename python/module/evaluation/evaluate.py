# -*- coding: utf-8 -*- 
import os
import sys
import importlib
import random

#print("args is ",sys.argv);# ["evaluate.py", "python/hellopython", "赵勇","python/hellopython/赵勇/Score.txt"] 
#sys.argv=["",".","zy","score.txt"]

herepath=os.path.split(sys.argv[0])[0]
topicFolder=os.path.join(herepath,"..")

if len(sys.argv)<4:
    exit(0)

if not os.access('{}/{}/person_package/__init__.py'.format(topicFolder,sys.argv[2]),os.R_OK):
    os.system('echo "[D]({}/evaluation/none_init.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

if not os.access('{}/{}/plot.png'.format(topicFolder,sys.argv[2]),os.R_OK):
    os.system('echo "[D]({}/evaluation/none_plot.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)


fd = os.popen('python {}/{}/test_package.py'.format(herepath,sys.argv[2]))
result = fd.read()
fd.close()

if result == "[1, 2, 8, 9, 10, 11, 13]":
   os.system('echo "[S]({}/{}/test_package.py)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
else:
   os.system('echo "[D]({}/evaluation/sort.md)" > {}'.format(sys.argv[1],sys.argv[3]))







