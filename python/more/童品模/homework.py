# -*- coding: utf-8 -*-
# coding:utf-8
from __future__ import print_function
import time
import sys

# 采用递归的方式实现
def Fibonacci_Recursion_tool(n):
    if n == 1:
        return 1
    elif n == 2:
        return 1
    else:
        return Fibonacci_Recursion_tool(n-1)+Fibonacci_Recursion_tool(n-2)

def Fibonacci_Recursion(n):
    result_list = []
    for i in range(1, n + 1):
        result_list.append(Fibonacci_Recursion_tool(i))
    return result_list

# 循环计算每项的值
def Fibonacci_Loop_tool(n):
    a = 1
    b = 1
    n = int(input())
    if n<=2 :
        print(1)
    else:
        print (1,1,end=' ')
        for i in range(1,n-1):
            a,b = b,a+b
            print (b)


def Fibonacci_Loop(n):
    result_list = []
    a, b = 0, 1
    i = 0
    while i < n:
        result_list.append(b)
        a, b = b, a + b
        i += 1
    return result_list

# 采用迭代的方式实现
def Fibonacci_Yield_tool(n):
    a, i, b = 0, 0, 1
    while i < n:
        yield b #yield就是 return 返回一个值，并且记住这个返回的位置，下次迭代就从这个位置后开始。
        a, b = b, a + b
        i += 1


def Fibonacci_Yield(n):
    return list(Fibonacci_Yield_tool(n))

# 性能比较
def Test_Fibonacci(n):
    F_R = Fibonacci_Recursion(n)
    F_L = Fibonacci_Loop(n)
    F_Y = Fibonacci_Yield(n)
    return F_R, F_L, F_Y

a, b, c = Test_Fibonacci(10)
sys.stdout.write(str(a)+str(b)+str(c))
