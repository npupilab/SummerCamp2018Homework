def sort_tool(var_list, start, end):
    if start < end:
        i, j = start, end
        # 设置基准数
        base = var_list[i]

        while i < j:
            # 如果列表后边的数,比基准数大或相等,则前移一位直到有比基准数小的数出现
            while (i < j) and (var_list[j] >= base):
                j = j - 1

            # 如找到,则把第j个元素赋值给第个元素i,此时表中i,j个元素相等
            var_list[i] = var_list[j]

            # 同样的方式比较前半区
            while (i < j) and (var_list[i] <= base):
                i = i + 1
            var_list[j] = var_list[i]
        # 做完第一轮比较之后,列表被分成了两个半区,并且i=j,需要将这个数设置回base
        var_list[i] = base

        # 递归前后半区
        sort_tool(var_list, start, i - 1)
        sort_tool(var_list, j + 1, end)
    return var_list

def sort(var_list):
    var_list = sort_tool(var_list, 0, len(var_list) - 1)
    return var_list

if __name__ == "__main__":
    result=sort([0,3,4,2,6])
    if(result==[0,2,3,4,6]):
        print("PASS")
    else:
        print("failed")

