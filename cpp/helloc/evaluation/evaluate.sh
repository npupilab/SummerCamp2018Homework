#!/bin/bash

#####################  Build in Things   ######################
#. $Function_Top/Include/Enviroment_Config.inc
Call_Path=$(pwd) #Where this call from?
#Where is here?
if [ -n "$Function_Path" ];then
	Here_Path=$Function_Path
else
	Here_Path=$(dirname $(readlink -f $0))
fi
topicFolder=$Here_Path/..
topic=$1
name=$2
BuildDir=$Call_Path/$topic
scoreFile=$3

mkdir -p $BuildDir/$name

if [ ! -f "$topicFolder/$name/main.c" ];then
    echo "[D]($topic/evaluation/no_main.md)"> "$scoreFile"
    exit 0
fi

g++ -o $BuildDir/$name/a.out $topicFolder/$name/main.c

if [ ! -f "$BuildDir/$name/a.out" ];then
    echo "[C]($topic/evaluation/compile_failed.md)"> "$scoreFile"
    exit 0
fi

for ((a=0;a<1000;a++));do
  var[0]="0"
  for ((i=1;i<100;i++));do var[$i]="$(( (RANDOM % 1000) + 1 ))";done
  var[100]="0"

  #echo "$BuildDir/$name/a.out ${var[@]}"
  i="$($BuildDir/$name/a.out ${var[@]})"
  #echo "output: $i"
  reg='([0-9]+)'
  if [[ $i =~ $reg ]];then
    i="${BASH_REMATCH[1]}";
  else
    echo "[B]($topic/evaluation/wrong_output.md)"> "$scoreFile"
    exit 0
  fi

  if [ -z "$i" ];then
    echo "No output from $BuildDir/$name/a.out ${var[@]}"
    echo "[B]($topic/evaluation/app_no_output.md)"> "$scoreFile"
    exit 0
  fi
  #echo "${var[$i]}" -le "${var[$((i-1))]}"
  if [ "${var[$i]}" -lt "${var[$((i-1))]}" ];then
    echo "[B]($topic/evaluation/wrong_output.md)"> "$scoreFile"
    exit 0
  fi

  #echo "${var[$i]}" -le "${var[$((i+1))]}"
  if [ "${var[$i]}" -lt "${var[$((i+1))]}" ];then
    echo "[B]($topic/evaluation/wrong_output.md)"> "$scoreFile"
    exit 0
  fi
done

echo "[S]($topic/$name/main.c)"> "$scoreFile";
