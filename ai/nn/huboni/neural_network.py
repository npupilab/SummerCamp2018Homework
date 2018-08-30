#!/usr/bin/env python
# -*_ coding:utf-8 -*-

'''
A neural_neywork learning algorithm example using TensorFlow library.
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
#decay_rate = 0.96
#decay_steps = 100
training_epochs = 100
batch_size = 200
display_step = 1

# set the tf Graph Input & set the model weights
# TO DO
# 开始准确率一直0.8左右，更改了激活函数后准确率高达0.97
x = tf.placeholder(dtype=tf.float32,shape=[None,784])
y = tf.placeholder(dtype=tf.float32,shape=[None,10])
w1 = tf.get_variable("w1",shape=[784,128],initializer=tf.contrib.layers.xavier_initializer())
b1 = tf.Variable(tf.random_normal([128]))
l1 = tf.nn.leaky_relu(tf.matmul(x,w1)+b1)

w2 = tf.get_variable("w2",shape=[128,128],initializer=tf.contrib.layers.xavier_initializer())
b2 = tf.Variable(tf.random_normal([128]))
l2 = tf.nn.leaky_relu(tf.matmul(l1,w2)+b2)

w3 = tf.get_variable("w3",shape=[128,10],initializer=tf.contrib.layers.xavier_initializer())
b3 = tf.Variable(tf.random_normal([10]))
pred = tf.nn.leaky_relu(tf.matmul(l2,w3)+b3)

# Construct the model
# TO DO
# a = tf.nn.relu(tf.matmul(x,w1)+b1)
# pred = tf.nn.relu(tf.matmul(a,w2) + b2)

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
