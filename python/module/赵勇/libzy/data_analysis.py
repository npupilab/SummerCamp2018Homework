#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()
        self.sequence = map(int,lines[1].strip().split(' '))
        self.x= map(float,lines[3].split(":")[1].strip().split(','))
        self.y= map(float,lines[4].split(":")[1].strip().split(','))

    def findMaxSubSequence(self):
        if len(self.sequence) <=1:
            return self.sequence

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
        figure=plt.figure()
        a=figure.add_subplot(111)
        a.plot(self.x,self.y,'r-',label='XY2D')
        #plt.figlegend(loc='upper left')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        savefig('./plot.png')











