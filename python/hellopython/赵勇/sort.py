def simplesort(var_list):
    for i in range(len(var_list)):
        for j in range(i+1,len(var_list)):
            if var_list[i]>var_list[j]:
                var_list[i],var_list[j]=var_list[j],var_list[i]
    return var_list

def bubbleSort(var_list):
    for i in range(len(var_list)-1):
        for j in range(len(var_list)-i-1):
            if var_list[j]>var_list[j+1]:
                var_list[j],var_list[j+1]=var_list[j+1],var_list[j]
    return var_list

#quick sort
def quickSort(L, low, high):
    i = low 
    j = high
    if i >= j:
        return L
    key = L[i]
    while i < j:
        while i < j and L[j] >= key:
            j = j-1                                                             
        L[i] = L[j]
        while i < j and L[i] <= key:    
            i = i+1 
        L[j] = L[i]
    L[i] = key 
    quickSort(L, low, i-1)
    quickSort(L, j+1, high)
    return L

def sort(var_list):
    return quickSort(var_list,0,len(var_list)-1)


if __name__ == "__main__":
    result=sort([0,3,4,2,6])
    if(result==[0,2,3,4,6]): 
        print("PASS");
    else:
        print("Failed")
    


