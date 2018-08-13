# -*- coding: utf-8 -*- 
from __future__ import print_function
import time
import sys
# 采用递归的方式实现
def Fibonacci_Recursion_tool(n):
    if n == 1:
        return 1
    elif n== 2:
        return 1
    else:
        return Fibonacci_Recursion_tool(n-1)+Fibonacci_Recursion_tool(n-2)
    
def Fibonacci_Recursion(n):
    result_list = []
    for i in range(1, n + 1): result_list.append(Fibonacci_Recursion_tool(i))
    return result_list

# 循环计算每项的值
def Fibonacci_Loop_tool(n):
    print("Execuse Me?")    

def Fibonacci_Loop(n):
    result_list = []
    a, b = 0, 1
    while n > 0:
        result_list.append(b)
        a, b = b, a + b
        n -= 1
    return result_list

# 采用迭代的方式实现
def Fibonacci_Yield_tool(n):
    a, i, b = 0, 0, 1
    while i < n:
        yield b
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
