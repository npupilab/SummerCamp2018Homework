#include <stdio.h>
#include <stdlib.h>


int find_peak(int n1, int n2, int n, int *arr)
{
    int m, mp1, mm1;
    int p1, p2;
    
    if( n1 == n2 ) return -1;
    
    m = (int)( (n1 + n2) / 2 );
    mp1 = m + 1;
    mm1 = m - 1;
    
    if( mp1 >= n ) return -1;
    if( mm1 < 0 ) return -1;
    
    if( arr[m] > arr[mp1] && arr[m] > arr[mm1] ) 
    {
        return m;
    }
    else
    {
        p1 = find_peak(n1, m, n, arr);
        if( p1 > 0 ) return p1;
        
        p2 = find_peak(m, n2, n, arr);
        if( p2 > 0 ) return p2;
        
        return -1;
    }
}

int main(int argc, char *argv[])
{
    int i, n;
    int *arr;
    int p_idx;
    
    // get input arguments
    n = argc;
    arr = (int*)malloc(sizeof(int)*(n-1));
    
    for(i=1; i<n; i++) 
    { 
        arr[i-1] = atoi(argv[i]);
    }   
        
    // get peak index
    p_idx = find_peak(0, n-1, n-1, arr);
    
    printf("%d\n", p_idx);
    
    // free array
    free(arr);
    arr = NULL;
    
    return 0;
}


