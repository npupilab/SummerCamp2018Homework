#!/usr/bin/python
# -*- coding:utf8 -*-

def bubble_sort(lists):
    count = len(lists)
    for i in range(0, count):
        for j in range(i + 1, count):
            if lists[i] > lists[j]:
                lists[i], lists[j] = lists[j], lists[i]
    return lists

if __name__ == "__main__":
    result = bubble_sort([0, 3, 4, 2, 6])
    if (result == [0, 2, 3, 4, 6]):
        print("PASS");
    else:
        print("Failed")

