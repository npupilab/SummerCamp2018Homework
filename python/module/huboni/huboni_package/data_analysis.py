#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            self.lines = f.readlines()

        self.result = []
        for line in self.lines:
            if(line[0] == "#" or line[0] == "\n"):
                continue
            else:
                self.result.append(line)
        self.result[0] = self.result[0].split("\n")[0].strip().split(" ")
        self.result[1] = self.result[1].split(":")[1].strip().split(",")
        self.result[2] = self.result[2].split(":")[1].strip().split(",")





    def findMaxSubSequence(self):
        self.result[0] = list(map(int, self.result[0]))
        longest = [1] * len(self.result[0])

        for i in range(1, len(self.result[0])):
            for j in range(0, i):
                if self.result[0][i] > self.result[0][j] and longest[i] < longest[j] + 1:
                    longest[i] = longest[j] + 1

        subseq = list()
        num = 1
        for i in range(0, len(longest)):
            if longest[i] == num:
                subseq.append(self.result[0][i])
                if num == max(longest):
                    break
                else:
                    num += 1

        return(subseq)


    def plotXY(self):
        list1 = map(float,self.result[1])
        list2 = map(float,self.result[2])

        figure = plt.figure(1)
        plt.plot(list(list1),list(list2),'r-',label='XY2D')
        plt.figlegend(loc='upper left')
        plt.xlabel('x')
        plt.ylabel('y')
        plt.xlim(0,10)
        plt.ylim(0,10)
        plt.axis('equal')

        savefig('/home/huboni/projects/huboni_module/plot.png')
        # plt.show()
