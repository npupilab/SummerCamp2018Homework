#!/bin/bash

# init a array
a=(1 2 3 4)

# get a item
echo "a[0] = ${a[0]}"

# get an item with index variable
c=2
echo "a[$c] = ${a[$c]}"

# get array size
echo "sizeof(a) = ${#a[@]}"

# get all items
echo "${a[@]}"

