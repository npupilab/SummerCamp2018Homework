#!/usr/bin/env python
# -*_ coding:utf-8 -*-
#from __future__ import pritn_function
#import time
# import sys
# n=int(sys.argv[1])
# 循环
my_list = [1, 1]
def Fibonacci_Loop(my_list, n):
    for i in range(2, n):
        add = my_list[i - 1] + my_list[i - 2]
        my_list.append(add)

Fibonacci_Loop(my_list, 10)
print(my_list)

# 递归
def Fibonacci_Recursion_tool(n):
    if n < 1:
        return -1
    elif n == 1 or n == 2:
        return 1
    else:
        return Fibonacci_Recursion_tool(n - 1) + Fibonacci_Recursion_tool(n - 2)
def Fibonacci_Recursion(n):
    result_list = []
    for i in range(1, n + 1):
        result_list.append(Fibonacci_Recursion_tool(i))
    return result_list
print(Fibonacci_Recursion(10))

# 迭代
def Fibonacci_Yield(max):
    n, a, b = 0, 0, 1
    while n < max:
        print(b)
        a, b = b, a + b
        n = n + 1
    return "done"
print(Fibonacci_Yield(10))

# 性能比较
