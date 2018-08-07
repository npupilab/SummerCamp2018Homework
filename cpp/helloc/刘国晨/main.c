#include <stdio.h>

int main(int argc, char** argv)
{
    int first_num = (int)argv[1][0];
    int mid_num = (int)argv[2][0];
    int last_num = (int)argv[3][0];
    int index = 2;

    while(index != argc - 2)
    {
        if(first_num < mid_num && mid_num > last_num)
        {
            return index;
        }

        index++;
        first_num = mid_num;
        mid_num = last_num;
        last_num = argv[index + 2][0];
    }

    return 0;
}
