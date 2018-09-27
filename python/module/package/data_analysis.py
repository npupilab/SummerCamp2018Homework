#!/usr/bin/python
# -*- coding:utf8 -*-
import matplotlib.pyplot as plt
from matplotlib.pyplot import savefig


class DataAnalysis(object):
	def __init__(self, filename):

		with open(filename ,'r') as f:
			lines = f.readlines()
			self.seq = lines[1] 
			self.x = lines[3]
			self.y = lines[4]


	def findMaxSubSequence(self):

		int_seq = list(map(int, self.seq.split()))    #split seq to list which type is int 

		index_seq = [0 for i in range(len(int_seq))]	#[0]*len(int_seq)
		for i in range(1,len(int_seq)):
			for j in range(0, i):
				if int_seq[i] > int_seq[j] and index_seq[i] <= index_seq[j]:
					index_seq[i] += 1	

		for i in range(1, len(int_seq)):
			if int_seq[0] > int_seq[i]:
				index_seq[0] += 1	#judge whether the first one of int_seq is minimum

		index_min = min(index_seq)

		subseq = []
		for i in range(len(int_seq)):
			if index_seq[i] == index_min:
				subseq.append(int_seq[i])
				index_min += 1

		return subseq


	def plotXY(self):
		self.x = self.x[2:]
		self.x = self.x.strip()
		self.x = self.x.split(',')
		self.x = list(map(float, self.x))
		self.y = self.y[2:]
		self.y = self.y.strip()
		self.y = self.y.split(',')
		self.y = list(map(float, self.y))


		plt.plot(self.x, self.y)
		savefig('./plot.png')
		plt.show()


calc_subseq = DataAnalysis('data.txt')
calc_subseq.plotXY()
print(calc_subseq.findMaxSubSequence())













