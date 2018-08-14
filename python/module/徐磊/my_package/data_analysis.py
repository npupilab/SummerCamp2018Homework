#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r') as f:
            lines = f.readlines()
            str_array = lines[1].split();
            self.array = []
            self.x =[]
            self.y = []
            for i in range(len(str_array)):
                self.array.append(int(str_array[i]))
            str_x = lines[3]
            str_y = lines[4]
            temp_x = str_x[2:-1]
            temp_y = str_y[2:-1] 
            str_x = (temp_x.strip()).split(',')
            str_y = (temp_y.strip()).split(',')
            for i in range(len(str_x)):
                self.x.append(float(str_x[i]))
                self.y.append(float(str_y[i]))


    def findMaxSubSequence(self):
        '''
        todo latter, read books to learn LIS
        '''
        arr = self.array
        n = len(self.array)
        m = [0]*n
        for x in range(n-2,-1,-1):
            for y in range(n-1,x,-1):
                if arr[x] < arr[y] and m[x] <= m[y]:
                    m[x] += 1
            max_value = max(m)
            result = []
            for i in range(n):
                if m[i] == max_value:
                    result.append(arr[i])
                    max_value -= 1
        return result

    def plotXY(self):
        plt.plot(self.x,self.y)
        savefig('./plot.png')











