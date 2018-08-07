#include <stdio.h>
#include <stdlib.h>

int find_peak(int *array ,int pos, int length)
{
	if(pos == length-1){
		return -1;	
	}
	else
	{
		if(array[pos] > array[pos-1] && array[pos] > array[pos +1 ]) return pos;
		else return find_peak(array,pos+1,length);
	}
}

int main(int argc, char** argv)
{
	if(argc < 2)
	{
		printf("Please Input some numbers at least 3!");
		return -1;
	}
	else if(argc < 4)
	{
		printf("Please at least 3 numbers !");
		return -1;
	}
	int *a=NULL;
	a = (int*)malloc((argc-1)*sizeof(int));
	for(int i=1; i < argc; i++)  a[i-1] = atoi(argv[i]);

	
	printf("Find Result %d \n",find_peak(a,1,argc-1));
	free(a);
	return 0;

}


