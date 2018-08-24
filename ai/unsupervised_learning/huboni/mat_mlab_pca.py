#!/usr/bin/env python
# -*_ coding:utf-8 -*-
#matplotlib.malb库实现
from matplotlib.mlab import PCA as mlabPCA
import numpy as np
from matplotlib import pyplot as plt

np.random.seed(123456)  # this can be avoid to use a smaller seed

mu_vec1 = np.array([0,0,0])
cov_mat1 = np.array([[1,0,0],[0,1,0],[0,0,1]])
class1_sample = np.random.multivariate_normal(mu_vec1,cov_mat1,20).T
assert class1_sample.shape == (3,20)

mu_vec2 = np.array([1,1,1])
cov_mat2 = np.array([[1,0,0],[0,1,0],[0,0,1]])
class2_sample = np.random.multivariate_normal(mu_vec2,cov_mat2,20).T
assert class2_sample.shape == (3,20)

all_samples = np.concatenate((class1_sample,class2_sample),axis=1)
assert all_samples.shape == (3,40)


mlab_pca = mlabPCA(all_samples.T)
print('mlab_pca :\n',mlab_pca.Wt)

plt.plot(mlab_pca.Y[0:20,0],mlab_pca.Y[0:20,1],'o',markersize = 7,color='blue',alpha=0.5,label='class1')
plt.plot(mlab_pca.Y[20:40,0],mlab_pca.Y[20:40,1],'o',markersize = 7,color='red',alpha=0.5,label='class2')
plt.xlim([-4,4])
plt.ylim([-4,4])
plt.xlabel('x_values')
plt.ylabel('y_values')
plt.legend()
plt.title('transformed samples')
plt.show()
