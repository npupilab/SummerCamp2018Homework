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
    echo "[D](no main.c)"> "$scoreFile"
    exit 0
fi

g++ -o $BuildDir/$name/a.out $topicFolder/$name/main.c

if [ ! -f "$BuildDir/$name/a.out" ];then
    echo "[C](compile failed)"> "$scoreFile"
    exit 0
fi

for ((a=0;a<1000;a++));do
  var[0]="-1"
  for ((i=1;i<1000;i++));do var[$i]="$(( (RANDOM % 1000) + 1 ))";done
  i="$($BuildDir/$name/a.out ${var[@]})"

  if [ ! -n "$i" ];then
    echo "[B](app no output)"> "$scoreFile"
  fi

  if [ "${var[$i]}" -le "${var[$((i-1))]}" ];then
    echo "[B](wrong output)"> "$scoreFile"
  done


  if [ "${var[$i]}" -le "${var[$((i+1))]}" ];then
    echo "[B](wrong output)"> "$scoreFile"
  done
done

echo "[S]($topic/$name/main.c)"> "$scoreFile";
