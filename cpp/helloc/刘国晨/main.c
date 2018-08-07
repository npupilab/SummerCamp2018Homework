#include <stdio.h>
#include <stdlib.h>

int main(int argc, char** argv)
{
    int first_num = atoi(argv[1]);
    int mid_num = atoi(argv[2]);
    int last_num = atoi(argv[3]);
    int index = 2;

    while(index != argc - 2)
    {
        if(first_num < mid_num && mid_num > last_num)
            return index-1;

        index++;
        first_num = mid_num;
        mid_num = last_num;
        last_num = atoi(argv[index + 2]);
    }

    return 0;
}
