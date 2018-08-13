#include<stdlib.h>
#include <iostream>
#include<vector>
#include<string>
#include<sstream>

int main(int argc,char **argv){
    int n = argc -1;
    int * data = new int[n];

    if(argc < 2){
        std::cout<<"please input an array:"<<std::endl;
        return -1;
    }


    for(int i = 1;i < argc;i++){
        int a;
        a = atoi(argv[i]);
        data[i - 1] = a;

    }

    int start = 0,end = n - 1,temp;

    while(end >=start){
        int mid = (start + end)/2;
//        std::cout<<"data[mid]:"<<data[mid]<<",data[mid-1]:"<<data[mid-1]<<",data[mid+1]:"<<data[mid+1]<<std::endl;
        if((data[mid]>data[mid-1])&&(data[mid]>data[mid+1])){

            temp = data[mid];
            break;
        }
        else if(data[mid] <= data[mid-1]){
            end = mid - 1;
        }
        else if(data[mid] <= data[mid+1]){
            start  = mid + 1;
        }


    }
    for(int index = 0;index < n;index++){
        if(data[index] == temp)
            std::cout<<index<<std::endl;

    }

    delete[] data;

    return 0;
}