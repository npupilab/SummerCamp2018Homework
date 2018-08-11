#!/usr/bin/python2.7
# -*- coding:utf-8 -*-
import sys 

reload(sys)
sys.setdefaultencoding('utf8')


from person_package.data_analysis import DataAnalysis


filename = 'data.txt'

analysis=DataAnalysis(filename)

I=analysis.plotXY()

subseq = analysis.findMaxSubSequence()

if subseq == [1, 2, 8, 9, 10, 11, 13]:
    print('PASS')
else:
    print('FAILURE')

sys.stdout.write(str(subseq))









