#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()
            self.seq = lines[1].strip().split(' ')
            self.xy = {"x":[],"y":[]}

            self.x = lines[3].split(":")[1].strip().split(',')
            self.y = lines[4].split(":")[1].strip().split(',')

            self.x = map(float, x)
            self.y = map(float, y)

            self.seq = [int(i) for i in self.seq]
            self.xy["x"] = x
            self.xy["y"] = y

    def findMaxSubSequence(self):

        list = self.seq
        num = len(list)
        m = [0] * num
        self.max_seq_num = 1
        for x in range(num - 2, -1, -1):
            m_x = [0] * num
            for y in range(num - 1, x, -1):
                if list[x] < list[y]:
                    m_x[y] = m[y]
            m[x] = (max(m_x) + 1)
        max_seq_num = max(m)
        subseq = []
        for i in range(num):
            if m[i] == maxall:
                subseq.append(list[i])
                max_seq_num -= 1

        return subseq


    def plotXY(self):
        plt.plot(self.x, self.y)
        savefig('./plot.png')
     #   plt.show()











