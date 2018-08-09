# -*- coding: utf-8 -*- 
import sys
import os

#print("args is ",sys.argv);# ["evaluate.py", "python/hellopython", "赵勇","python/hellopython/赵勇/Score.txt"] 

#hwpath = os.path.join(sys.argv[1], sys.argv[2])
#print(hwpath)

fd = os.popen('python ../{}/homework.py'.format(sys.argv[2]))
result = fd.read()
fd.close()

if result == "[1, 1, 2, 3, 5, 8, 13, 21, 34, 55][1, 1, 2, 3, 5, 8, 13, 21, 34, 55][1, 1, 2, 3, 5, 8, 13, 21, 34, 55]":
   os.system('echo "A" > ../{}/README.md'.format(sys.argv[2]))
