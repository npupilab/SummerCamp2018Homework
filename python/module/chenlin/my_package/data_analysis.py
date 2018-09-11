#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()
        self.seq=[int(i) for i in lines[1].strip().split(' ')]
        self.x=[float(i) for i in lines[3][2:].strip().split(',')]
        self.y=[float(i) for i in lines[4][2:].strip().split(',')]

    def findMaxSubSequence(self):
        subseq=[]
        Max_lists=[[self.seq[0]]]
        for i in range(1,len(self.seq)):
            for l in Max_lists:
                
                
        return subseq

    def plotXY(self):
        plt.plot(self.x,self.y)
        savefig('./plot.png')










