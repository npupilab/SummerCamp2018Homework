#!/usr/bin/env python
# -*_ coding:utf-8 -*-
def sort(var_list,start,end):
    #判断low是否小于high,如果为false,直接返回
    if start < end:
        i,j = start,end
        #设置基准数
        base = var_list[i]

        while i < j:
            #如果列表后边的数,比基准数大或相等,则前移一位直到有比基准数小的数出现
            while (i < j) and (var_list[j] >= base):
                j = j - 1

            #如找到,则把第j个元素赋值给第个元素i,此时表中i,j个元素相等
            var_list[i] = var_list[j]

            #同样的方式比较前半区
            while (i < j) and (var_list[i] <= base):
                i = i + 1
            var_list[j] = var_list[i]
        #做完第一轮比较之后,列表被分成了两个半区,并且i=j,需要将这个数设置回base
        var_list[i] = base

        #递归前后半区
        sort(var_list, start, i - 1)
        sort(var_list, j + 1, end)
    return var_list

if __name__ == "__main__":
    result=sort([0,3,4,2,6],0,4)
    if(result==[0,2,3,4,6]):
        print("PASS")
    else:
        print("failed")

