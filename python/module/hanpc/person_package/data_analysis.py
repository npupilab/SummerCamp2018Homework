#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()

        self.sequence = lines[1].strip().split(' ')
        self.xy = {"x":[],"y":[]}

        x = lines[3].split(":")[1].strip().split(',')
        y = lines[4].split(":")[1].strip().split(',')

        x = map(int, x)
        y = map(float, y)

        self.sequence= map(int, self.sequence)
        self.xy["x"] = x
        self.xy["y"] = y



    def findMaxSubSequence(self):

        ##todo 

        return subseq


    def plotXY(self):
         ##todo
        savefig('./plot.png')
        plt.show()











