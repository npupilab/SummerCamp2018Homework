# -*- coding: utf-8 -*- 
import sys
import os

#print("args is ",sys.argv);# ["evaluate.py", "python/hellopython", "赵勇","python/hellopython/赵勇/Score.txt"] 

#hwpath = os.path.join(sys.argv[1], sys.argv[2])
#print(hwpath)
herepath=os.path.split(sys.argv[0])[0]

if not os.access('{}/../{}/homework.py'.format(herepath,sys.argv[2]),os.R_OK):
    os.system('echo "[D]({}/evaluation/none.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

try:
    fd = os.popen('python {}/../{}/homework.py'.format(herepath,sys.argv[2]))
    result = fd.read()
    fd.close()
    if result == "[1, 1, 2, 3, 5, 8, 13, 21, 34, 55][1, 1, 2, 3, 5, 8, 13, 21, 34, 55][1, 1, 2, 3, 5, 8, 13, 21, 34, 55]":
       os.system('echo "[S]({}/{}/homework.py)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
    else:
       os.system('echo "[B]({}/evaluation/wrong.md)" > {}'.format(sys.argv[1],sys.argv[3]))
except SyntaxError:
    os.system('echo "[C]({}/evaluation/failed_run.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)
    

