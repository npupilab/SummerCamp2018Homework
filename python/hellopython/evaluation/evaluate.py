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

if not os.access('{}/{}/sort.py'.format(topicFolder,sys.argv[2]),os.R_OK):
    os.system('echo "[D]({}/evaluation/none.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

sys.path.append('{}/{}'.format(topicFolder,sys.argv[2]))

try:
    import sort
except ImportError:
    os.system('echo "[C]({}/evaluation/import.md)" > {}'.format(sys.argv[1],sys.argv[3]))
    exit(0)

nums=[]
for i in range(1000):
    nums.append(random.randint(0, 1000))

if(sorted(nums)==sort.sort(nums)):
    os.system('echo "[S]({}/{}/sort.py)" > {}'.format(sys.argv[1],sys.argv[2],sys.argv[3]))
else:
    os.system('echo "[B]({}/evaluation/sort.md)" > {}'.format(sys.argv[1],sys.argv[3]))






