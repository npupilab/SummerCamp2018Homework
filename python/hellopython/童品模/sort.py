#!/usr/bin/python2.7
#coding:utf-8

def bubbleSort(var_list):
    for i in range(len(var_list)):
        for j in range(i+1,len(var_list)):
            if var_list[j-1]>var_list[j]:
                var_list[j-1],var_list[j]=var_list[j],var_list[j-1]
    return var_list


def quickSort(L,left,right):
    i=left;
    j=right;
    if i>j:
        return L
#设置基准数
    key=L[i]
    while i<j:
        # 如果列表后边的数,比基准数大或相等,则前移一位直到有比基准数小的数出现
        while i<j and L[j] >= key:
            j=j-1
            #如果找到，就把第j个元素赋值给第i个元素；
        L[i]=L[j]
        # 如果列表前面的数,比基准数小或相等,则后移一位直到有比基准数大的数出现
        while i<j and L[i] <= key:
            i=i+1
        L[j]=L[i]
        #将base 返回
    L[i]=key
    quickSort(L,0,i-1)
    quickSort(L,j+1,right)
    return L

def sort(var_list):
  # return bubbleSort(var_list)
  return quickSort(var_list,0,len(var_list)-1)

if __name__ == "__main__":
    result=sort([0,3,4,2,6])
    if(result==[0,2,3,4,6]):
        print("PASS");
    else:
        print("Failed")
    # L=[2,34,78,90,76,23,54,16,89,63]
    # L=quickSort(L,0,len(L)-1)
    # print L




