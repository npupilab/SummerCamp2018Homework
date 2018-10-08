#include <stdio.h>
#include <stdlib.h>

int find(int left,int right , int a[])
{
	int mid = (left + right) / 2;

	if(left == right)
		return left;

	if(a[mid] > a[mid+1])
		return find(left, mid, a);
	else
		return find(mid+1, right, a);

}

int main(int argc, char **argv)
{
	int a[argc-1];

	for(int i = 1; i < argc; i++)
	{
		a[i-1] = atoi(argv[i]);
	}

	printf("%d\n",find(0, argc-2, a));

	return 0;
}
