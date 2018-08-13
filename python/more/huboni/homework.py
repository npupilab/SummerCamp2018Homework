#import sys                                             
#n=int(sys.argv[1])                                     
#循环                                                     
my_list = [1,1]                                         
def my_append(my_list,n):                               
    for i in range(2,n):                                
        add = my_list[i-1] + my_list[i-2]               
        my_list.append(add)                             
                                                        
my_append(my_list,10)                                   
print(my_list)                                          
                                                        
#递归                                                     
def fab_digui(n):                                       
    if n < 1:                                           
        return -1                                       
    elif n ==1 or n==2:                                 
        return 1                                        
    else:                                               
        return fab_digui(n-1)+fab_digui(n-2)            
                                                        
def digui_append(n):                                    
    result_list=[]                                      
    for i in range(1,n+1):                              
        result_list.append(fab_digui(i))                
    return result_list                                  
                                                        
print(digui_append(10))                                 
                                                        
#迭代                                                     
def fib(max):                                           
    n, a, b = 0, 0, 1                                   
    while n < max:                                      
        print(b)                                        
        a, b = b, a + b                                 
        n = n + 1                                       
    return "done"                                       
                                                        
                                                        
print(fib(10))                                                                        
