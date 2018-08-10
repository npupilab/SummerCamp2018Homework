#include <stdio.h>
#include <stdlib.h>

int search(int left, int right, char** argv)
{
    int mid = (right + left) / 2;

    if( (atoi(argv[mid]) > atoi(argv[mid - 1])) && (atoi(argv[mid]) > atoi(argv[mid + 1])))
        return mid - 1;
    else if(atoi(argv[mid + 1]) > atoi(argv[mid]))
        search(mid, right, argv);
    else if(atoi(argv[mid - 1]) > atoi(argv[mid]))
        search(left, mid, argv);
}

int main(int argc, char** argv)
{
    int mid_index = argc / 2;
    if(mid_index == 0)
    {
        printf("none answer\n");
        return 0;
    }
    if(mid_index == 1)
    {
        printf("%i", 0);
        return 0;
    }

    int answer = search(1, argc - 1, argv);
    printf("%i", answer);

    return 0;
}
