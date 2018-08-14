def quickSort(list1, left, right):
    if left >= right:
        return list1
    i = left
    j = right
    temp = list1[i]
    while i != j:
        while i < j and list1[j] >= temp:
            j = j-1
        list1[i] = list1[j]
        while i < j and list1[i] <= temp:
            i = i+1
        list1[j] = list1[i]
    list1[i] = temp
    quickSort(list1, left, i-1)
    quickSort(list1, i+1, right)
    return list1

def sort(list1):
    return quickSort(list1,0,len(list1)-1)

if __name__ == "__main__":
    result = sort([0, 3, 4, 2, 6])
    if (result == [0, 2, 3, 4, 6]):
        print("PASS");
    else:
        print("Failed")
