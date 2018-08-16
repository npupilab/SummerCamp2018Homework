#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()

            self.sequence = lines[1].strip().split(' ')
            self.x = lines[3].split(":")[1].strip().split(',')
            self.y = lines[4].split(":")[1].strip().split(',')
            self.x = map(float, self.x)
            self.y = map(float, self.y)
            self.sequence = map(int, self.sequence)




    def findMaxSubSequence(self):
        list = self.sequence
        num = len(list)
        m=[0]*num
        maxall = 1
        for x in range(num-2, -1,-1):
            m_x = [0] * num
            for y in range(num-1, x,-1):
                if list[x] < list[y]:
                    m_x[y] = m[y]
            m[x] = (max(m_x) + 1)
        maxall = max(m)
        subseq = []
        for i in range(num):
            if m[i] == maxall:
                subseq.append(list[i])
                maxall -= 1

        return subseq




    def plotXY(self):
        plt.plot(self.x,self.y)
        savefig('./plot.png')





'''    def findMaxSubSequence(self):
        list = self.sequence
        num = len(list)
        m=[0]*num
        maxall = 1
        for x in range(1, num):
            m_x = [0] * num
            for y in range(0, x):
                if list[x] > list[y]:
                    m_x[y] = m[y]
            m[x] = (max(m_x) + 1)
        maxall = max(m)
        subseq = []
        for i in range(num-1,-1,-1):
            if m[i] == maxall:
                subseq.append(list[i])
                maxall -= 1
        subseq.reverse()
        return subseq'''
