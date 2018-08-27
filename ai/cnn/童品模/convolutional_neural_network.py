#!/usr/bin/python2.7
#coding:utf-8

'''
A cnn_mnist learning algorithm example using TensorFlow library.
This example is using the MNIST database of handwritten digits
(http://yann.lecun.com/exdb/mnist/)
Author:
'''
#include convolution layer,pooling layer,full connected layer;
from __future__ import print_function

import tensorflow as tf
import random

# Import MNIST data
from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("../Mnist_data/", one_hot=True)
print(mnist)

tf.set_random_seed(777)

# Parameters setting
learning_rate = 0.0001
training_epochs = 100 
batch_size = 100   
display_step=1

# set the tf Graph Input & set the model weights

x=tf.placeholder(dtype=tf.float32,shape=[None,784],name='input_x')
x_img=tf.reshape(x,[-1,28,28,1])

y=tf.placeholder(dtype=tf.float32,shape=[None,10],name='input_y')

#set layers
W1=tf.Variable(tf.random_normal([3,3,1,32],stddev=0.01))   
L1=tf.nn.conv2d(x_img,W1,strides=[1,1,1,1],padding='SAME')
L1=tf.nn.relu(L1)
L1=tf.nn.max_pool(L1,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

W2=tf.Variable(tf.random_normal([3,3,32,64],stddev=0.01))   
L2=tf.nn.conv2d(L1,W2,strides=[1,1,1,1],padding='SAME')
L2=tf.nn.relu(L2)
L2=tf.nn.max_pool(L2,ksize=[1,2,2,1],strides=[1,2,2,1],padding='SAME')

L2=tf.layers.dense(inputs=L2,units=1024,activation=None) #
L2_flat=tf.reshape(L2,[-1,7*7*1024])

W3=tf.get_variable("W3",shape=[7*7*1024,10],initializer=tf.contrib.layers.xavier_initializer())

b=tf.Variable(tf.random_normal([10]))

pred=tf.matmul(L2_flat,W3)+b


# Construct the model
# Minimize error using cross entropy & set the gradient descent
#tf.nn.softmax_cross_entropy_with_logits(logits=L3,labels=y)求出来的是一个vector,n_sample*10,求平均变成一个数；
cost=tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=pred,labels=y))
# cost=tf.reduce_mean(-tf.reduce_sum(y*tf.log(pred),reduction_indices=1)) #交叉熵，reducion_indices=1横向求和
optimizer=tf.train.AdamOptimizer(learning_rate).minimize(cost)


# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()

# Start training
with tf.Session() as sess:

    # Run the initializer
    #
    sess.run(init)

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0
        total_batch = int(mnist.train.num_examples/batch_size)
        # Loop over all batches
        for i in range(total_batch):
            batch_xs, batch_ys = mnist.train.next_batch(batch_size)
            # Run optimization op (backprop) and cost op (to get loss value)
            _, c = sess.run([optimizer, cost], feed_dict={x: batch_xs,
                                                          y: batch_ys})
            # Compute average loss
            avg_cost += c / total_batch
        # Display logs per epoch step
        if (epoch+1) % display_step == 0:
            print("Epoch:", '%04d' % (epoch+1), "cost=", "{:.9f}".format(avg_cost))

    print("Learning Finished!")

    # Test model
    correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
    # Calculate accuracy
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))  # tf.cast(x,dtype)将x转换为dtype类型；
    print("Accuracy:", accuracy.eval({x: mnist.test.images, y: mnist.test.labels}))

