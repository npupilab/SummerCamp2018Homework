#!/usr/bin/python27
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

        x = map(float, x)
        y = map(float, y)

        self.sequence=[int(i) for i in self.sequence]

        self.xy["x"] = x
        self.xy["y"] = y


    def findMaxSubSequence(self):

        list= self.sequence

        n = len(list)
        m = [0] * n

        for x in range(n-2, -1, -1):
            for y in range(n-1, x, -1):
                if list[x] < list[y] and m[x] <= m[y]:
                    m[x] += 1
            max_value = max(m)

            subseq = []

            for i in range(n):
                if m[i] == max_value:
                    subseq.append(list[i])
                    max_value -= 1

        return subseq


    def plotXY(self):

        x=self.xy["x"]
        y=self.xy["y"]
        
        figure=plt.figure()
        ax=figure.add_subplot(111)
        ax.plot(x,y,'r-',label='XY2D')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        savefig('./plot.png')











