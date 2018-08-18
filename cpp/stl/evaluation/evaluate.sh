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

if [ ! -f "$topicFolder/$name/vector.h" ];then
    echo "[D]($topic/evaluation/none.md)"> "$scoreFile"
    exit 0
fi

g++ -o $BuildDir/$name/a.out $topicFolder/src/main.cpp -I$topicFolder/$name  -std=c++11

if [ ! -f "$BuildDir/$name/a.out" ];then
    echo "[C]($topic/evaluation/compile_failed.md)"> "$scoreFile"
    exit 0
fi

output="$($BuildDir/$name/a.out $topic $name $scoreFile)"


if [ ! -f "$scoreFile" ];then
    echo $output
    echo "[C]($topic/evaluation/run_failed.md)"> "$scoreFile"
    exit 0
fi

