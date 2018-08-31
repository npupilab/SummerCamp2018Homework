#!/usr/bin/env python
# -*_ coding:utf-8 -*-
'''
A convolutional neural network learning algorithm example using TensorFlow library.
This example is using the MNIST database of handwritten digits
(http://yann.lecun.com/exdb/mnist/)
Author:
'''

from __future__ import print_function

import tensorflow as tf
import random

# Import MNIST data
from tensorflow.examples.tutorials.mnist import input_data

mnist = input_data.read_data_sets("./Mnist_data/", one_hot=True)

tf.set_random_seed(777)
# Parameters setting
learning_rate = 0.65
# decay_rate = 0.96
# decay_steps = 100
training_epochs = 50
batch_size = 200
display_step = 1

# set the tf Graph Input & set the model weights
# TO DO
x = tf.placeholder(dtype=tf.float32,shape=[None,784],name='input-x')
y = tf.placeholder(dtype=tf.float32,shape=[None,10],name='input-y')
x_img=tf.reshape(x,[-1,28,28,1],name='tensor-x')

#set layers
#详细用法见百度，x_img必须为一个向量表示卷积的输入图像，第二个参数相当于卷积核，第三个参数卷积时在每一步的步长
#第四个参数只能为"SAME'"VALID'之一
#stddev 正态分布的标准差
#[3,3,1,32]分别为卷积核的高宽，输入的通道数，输出的通道数即这一层输出32个特征
W1=tf.Variable(tf.random_normal([3,3,1,32],stddev=0.01))
L1=tf.nn.conv2d(x_img,W1,strides=[1,1,1,1],padding='SAME')
L1=tf.nn.relu(L1)
L1=tf.nn.max_pool(L1,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

W2=tf.Variable(tf.random_normal([3,3,32,64],stddev=0.01))
L2=tf.nn.conv2d(L1,W2,strides=[1,1,1,1],padding='SAME')
L2=tf.nn.relu(L2)
L2=tf.nn.max_pool(L2,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

L2=tf.layers.dense(inputs=L2,units=10,activation=None)
L2_flat=tf.reshape(L2,[-1,7*7*10])

W3=tf.get_variable("W3",shape=[7*7*10,10],initializer=tf.contrib.layers.xavier_initializer())

b=tf.Variable(tf.random_normal([10]))

pred=tf.matmul(L2_flat,W3)+b

# Minimize error using cross entropy & set the gradient descent
#cost = tf.reduce_mean(-tf.reduce_sum(y*tf.log(pred),reduction_indices=1))
#注意求loss需要加上tf.reduce_mean 求交叉熵需要加上tf.reduce_sum
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred,labels=y))
# 梯度下降法选择最好的模型
optimizer = tf.train.AdadeltaOptimizer(learning_rate).minimize(cost)

# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()

# Start training
with tf.Session() as sess:

    # Run the initializer
    sess.run(init)

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0.
        total_batch = int(mnist.train.num_examples/batch_size)
        # Loop over all batches
        for i in range(total_batch):
            batch_xs, batch_ys = mnist.train.next_batch(batch_size)
            _, c = sess.run([optimizer, cost], feed_dict={x: batch_xs,
                                                          y: batch_ys})
            # Compute average loss
            avg_cost += c / total_batch
            # print(avg_cost)
        # # Display logs per epoch step
        if (epoch+1) % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost=", "{:.9f}".format(avg_cost))

    print("Optimization Finished!")

    # Test model
    correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
    # Calculate accuracy
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
    print("Accuracy:", accuracy.eval({x: mnist.test.images, y: mnist.test.labels}))
