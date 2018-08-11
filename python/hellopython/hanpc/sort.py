def sort_(var_list):
    # Add your implementation here
    var_list.sort()
    return var_list

def sort(var_list):
    # Add your implementation here
    for i in range(0, len(var_list)):
        for j in range(0, len(var_list)-i-1):
            if var_list[j] > var_list[j+1]:
                var_list[j], var_list[j+1] = var_list[j+1], var_list[j]

    return var_list


if __name__ == "__main__":
    result=sort([0,3,4,2,6])
    if(result==[0,2,3,4,6]): 
        print("PASS");
    else:
        print("Failed")
    


