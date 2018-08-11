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
#Who am I?
if [ -n "$Function_Name" ];then
	File_Name=$Function_Name.sh
else
	File_Name=${0##*/};
	Function_Name=${File_Name%.*};
fi

topicFolder=$Here_Path/..
topic=$1
name=$2
scoreFile=$3

if [ ! -f "$topicFolder/$name/README.md" ];then
    echo "[D]($topic/evaluation/none.md)"> "$scoreFile"
    exit 0
fi

  HEADER_REG='#+ +'
  TABLE_REG='\|[^\|]+\|'
  REF_REG='\[.*\]'
  CODE_REG='```'
  while read line
    do  
    if [[ $line =~ $HEADER_REG ]];then HEADER_EXIST="Y";fi
    if [[ $line =~ $TABLE_REG ]];then TABLE_EXIST="Y";fi
    if [[ $line =~ $REF_REG ]];then REF_EXIST="Y";fi
    if [[ $line =~ $CODE_REG ]];then CODE_EXIST="Y";fi
  done < $topicFolder/$name/README.md

if [ -z "$HEADER_EXIST" ];then
    echo "[C]($topic/evaluation/full.md)"> "$scoreFile"
    exit 0
fi


if [ -z "$TABLE_EXIST" ];then
    echo "[C]($topic/evaluation/full.md)"> "$scoreFile"
    exit 0
fi

if [ -z "$REF_EXIST" ];then
    echo "[C]($topic/evaluation/full.md)"> "$scoreFile"
    exit 0
fi

if [ -z "$CODE_EXIST" ];then
    echo "[C]($topic/evaluation/full.md)"> "$scoreFile"
    exit 0
fi
echo "[S]($topic/$name/README.md)" > "$scoreFile"
