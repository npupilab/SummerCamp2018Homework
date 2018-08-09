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

if [ ! -f "$topicFolder/$name/student.h" ];then
    echo "[D]($topic/evaluation/no_student.md)"> "$scoreFile"
    exit 0
fi

g++ -o $BuildDir/$name/a.out $topicFolder/src/main.cpp $topicFolder/src/inlinecheck.cpp -std=c++11 -I$topicFolder/$name  

if [ ! -f "$BuildDir/$name/a.out" ];then
    echo "[C]($topic/evaluation/compile_failed.md)"> "$scoreFile"
    exit 0
fi

$BuildDir/$name/a.out $topic $name $scoreFile
