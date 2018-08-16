#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()

        self.sequence = lines[1].strip().split(' ')
        self.sequence = [int(i) for i in self.sequence]
        self.x = [int(i) for i in lines[3].split(":")[1].strip().split(',')]
        self.y = [float(i) for i in lines[4].split(":")[1].strip().split(',')]

    def findMaxSubSequence(self):
        list= self.sequence

        n = len(list)
        m = [1] * n

        for x in range(1,n):
            for y in range (0,x):
                if list[x] > list[y] and m[x] <= m[y]:
                    m[x] += 1

            subseq = []
            max_value = max(m)

            for i in range(n-1,-1,-1):
                if m[i] == max_value:
                    subseq.append(list[i])
                    max_value -= 1
            subseq.reverse()
        return subseq

    def plotXY(self):
        figure=plt.figure()
        a=figure.add_subplot(111)
        a.plot(self.x,self.y,'r-',label='XY2D')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.axis('equal')
        savefig('./plot.png')











