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

if [ ! -f "$topicFolder/$name/README.md" ];then
    echo "[D]($topic/evaluation/no_upload.md)"> "$scoreFile"
    exit 0
fi

rep="$(cat $topicFolder/$name/README.md)"
reg='https://github.com/([^.]+)/SummerCamp2018Homework'
echo $rep
if [[ $rep =~ $reg ]]; then
username="${BASH_REMATCH[1]}"
else echo "[C]($topic/evaluation/content.md)"> "$scoreFile";exit 0
fi

# Collect fork information
if [ ! -f "$BuildDir/forks" ];then
cd $BuildDir
output='wget https://api.github.com/repos/npupilab/SummerCamp2018Homework/forks'
cd $Call_Path
fi

if [ ! -f "$BuildDir/forks" ];then
  echo "[A]($topic/evaluation/download.md)"> "$scoreFile"
  exit 0
fi
reg="\"full_name\": \"$username/SummerCamp2018Homework\""
while read line
  do  
  if [[ $line =~ $reg ]];then
    echo "[S]($topic/$name/README.md)"> "$scoreFile";exit 0;
  fi
done < "$BuildDir/forks"
echo "$reg not found"
echo "[B]($topic/evaluation/fork.md)"> "$scoreFile"
