def Fibonacci_Recursion_tool(n):

    if n <= 2:
        return 1
    if n > 2:
        return Fibonacci_Recursion_tool(n-1) + Fibonacci_Recursion_tool(n-2)

def Fibonacci_Recursion(n):
    result_list = []
    for i in range(1, n + 1): result_list.append(Fibonacci_Recursion_tool(i))
    return result_list


a = Fibonacci_Recursion(2)
print a
