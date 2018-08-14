#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig

class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            self.lines = f.readlines()

        i = 0
        self.result = list()
        for line in self.lines:
            if(line[0] == '#' or line[0] == '\n'):
                continue
            else:
                self.result.append(line)
        self.result[0] = self.result[0].split('\n')[0].strip().split(" ")
        self.result[1] = self.result[1].split(':')[1].strip().split(",")
        self.result[2] = self.result[2].split(':')[1].strip().split(",")

    def findMaxSubSequence(self):
        seq = list()
        max_num = int(self.result[0][0]) - 1
        for i in range(0, len(self.result[0])):
            if max_num < int(self.result[0][i]):
                max_num = int(self.result[0][i])
                seq.append(int(self.result[0][i]))
            else:
                continue

        return seq

    def plotXY(self):
        list1 = map(float, self.result[1])
        list2 = map(float, self.result[2])

        figure = plt.figure(1)
        plt.plot(list(list1), list(list2), 'r-', label='XY2D')
        plt.figlegend(loc='upper left')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim(0, 10)
        plt.ylim(0, 10)
        plt.axis('equal')

        savefig('./plot.png')










