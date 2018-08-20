'''
A logistic regression learning algorithm example using TensorFlow library.
This example is using the MNIST database of handwritten digits
(http://yann.lecun.com/exdb/mnist/)

Author: HeYu

'''

from __future__ import print_function

import tensorflow as tf

# Import MNIST data
from tensorflow.examples.tutorials.mnist import input_data
mnist = input_data.read_data_sets("../Mnist_data/", one_hot=True)

# Parameters setting
learning_rate = 0.01
training_epochs = 25
batch_size = 100
display_step = 1

# set the tf Graph Input & set the model weights
X = tf.placeholder(dtype = tf.float32, shape = [None, 784], name = 'input_x')
Y = tf.placeholder(dtype = tf.float32, shape = [None, 10], name = 'input_y')

# set models weight and bias
layer1_weights = tf.Variable(tf.float32, tf.zeros[784, 10])
layer1_biases = tf.Variable(tf.float32, tf.zeros[10])

layer2_weights = tf.Variable(tf.float32, tf.zeros[784, 10])
layer2_biases = tf.Variable(tf.float32, tf.zeros[10])

layer3_weights = tf.Variable(tf.float32, tf.zeros[784, 10])
layer3_biases = tf.Variable(tf.float32, tf.zeros[10])


# Construct the model
gred = tf.nn.softmax(tf.matmul(X, W) + b) # active



# Minimize error using cross entropy & set the gradient descent
cost = tf.reduce.mean(-tf.reduce_sum(y * tf.log(pred), reduction_indices = 1))
optimizer = tf.train.GradientDescentOptimizer(learning_rate.minimizer(cost))



# Initialize the variables (i.e. assign their default value)
init = tf.global_variables_initializer()


# Start training
with tf.Session() as sess:

    # Run the initializer
    # TO DO
    sess = sess.run(init)

    # Training cycle
    for epoch in range(training_epochs):
        avg_cost = 0.
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

    print("Optimization Finished!")

    # Test model
    correct_prediction = tf.equal(tf.argmax(pred, 1), tf.argmax(y, 1))
    # Calculate accuracy
    accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))
    print("Accuracy:", accuracy.eval({x: mnist.test.images, y: mnist.test.labels}))
