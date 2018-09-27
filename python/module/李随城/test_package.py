#!/usr/bin/python2.7
# -*- coding:utf-8 -*-
from package.data_analysis import DataAnalysis
#import *** 

import sys 

#reload(sys)
#sys.setdefaultencoding('utf8')

filename = 'data.txt'

analysis=DataAnalysis(filename)

I=analysis.plotXY()

subseq = analysis.findMaxSubSequence()

if subseq == [1,2,8,9,10,11,13] or subseq == [1,2,3,9,10,11,13] or subseq == [1,2,3,5,10,11,13] or subseq == [1,2,3,5,6,11,13]:
    print('PASS')
else:
    print('FAILURE')










