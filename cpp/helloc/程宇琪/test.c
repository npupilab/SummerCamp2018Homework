/*#include<stdio.h>
#include <stdlib.h>
#include<iostream>
#include<math.h>
int main(int argc, char** argv)
{
	int num=5;
	if(argc=1)
	num = atoi(argv[1]);
	std::cout<<ceil(0.5*num)<<std::endl;
	return 0;
}*/


#include<stdio.h>
#include<stdlib.h>


int findmax(int nu, int num, double* Arr)
{
    int indel = 0.5*num;
    if( Arr[indel] > Arr[indel+1] && Arr[indel] > Arr[indel-1] )
        return nu+indel;
    else if (Arr[indel] < Arr[indel+1])
        return findmax(nu+indel,num-indel , &Arr[indel]);
    else
        return findmax(nu,num-indel, Arr);


}

int main (int argc ,char** argv )
{

    double* arr = (double*)malloc(argc * sizeof(double));
    for(int i = 1; i < argc; ++i)
        arr[i-1] = atof ( argv[i]);
    printf("%d",findmax(0,argc , arr));


    return 0;
}

