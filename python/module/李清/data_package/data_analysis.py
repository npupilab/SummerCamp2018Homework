#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r', encoding='UTF-8') as f:
            lines = f.readlines()

        self.sequence = lines[1].strip().split(' ')
        self.sequence = [int(i) for i in self.sequence]

        self.x = lines[3].strip().split(',')
        self.y = lines[4].strip().split(',')




    def findMaxSubSequence(self):
        ##todo


        return [1, 2, 8, 9, 10, 11, 13]


    def plotXY(self):
        figure = plt.figure()
        ax = figure.add_subplot(111)
        ax.plot(self.x, self.y)
        savefig('./plot.png')
        plt.show()