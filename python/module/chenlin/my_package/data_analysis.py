#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
    def __init__(self, filename):

        with open(filename ,'r',encoding='UTF-8') as f:
            lines = f.readlines()
        self.seq=[int(i) for i in lines[1].strip().split(' ')]
        self.x=[float(i) for i in lines[3][2:].strip().split(',')]
        self.y=[float(i) for i in lines[4][2:].strip().split(',')]

    def findMaxSubSequence(self):
        subseq=[]
        X_list=self.seq.copy()
        X_list.sort()
        n=len(X_list)
        Max_lists=[[0]*(n+1) for i in range(n+1)]
        Dir_lists=[['l']*(n+1) for i in range(n+1)]
        for i in range(n):
            for j in range(n):
                if X_list[i] == self.seq[j]:
                    Max_lists[i+1][j+1]=Max_lists[i][j]+1
                else:
                    Max_lists[i+1][j+1]=max(Max_lists[i][j+1],Max_lists[i+1][j])
        i=j=n
        while i>0 and j>0:
            if Max_lists[i][j]==Max_lists[i][j-1]:
                j-=1
                continue
            if Max_lists[i][j]==Max_lists[i-1][j-1]+1:
                j-=1
                i-=1
                subseq.append(self.seq[j])
        subseq.reverse()                 
        return subseq

    def plotXY(self):
        plt.plot(self.x,self.y)
        savefig('./plot.png')










