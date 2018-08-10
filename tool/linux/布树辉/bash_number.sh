#!/bin/bash

# number arithmetic expansion
num1=10
num2=20

num11=$((num1 + num2))
num12=$(($num1 + $num2))       # also works
num13=$((num1 + 2 + 3))        # ...
num14=$[num1+num2]             # old, deprecated arithmetic expression syntax

echo "num11 = $num11"
echo "num12 = $num12"
echo "num13 = $num13"
echo "num14 = $num14"


# for with number
num=0
metab=0

for ((i=1; i<=2; i++)); do
    metab=$(($metab + 1))
    echo "$metab"
done
