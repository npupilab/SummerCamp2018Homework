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
BuildDir=$Call_Path/$topic/$name
scoreFile=$3

mkdir -p $BuildDir

if [ ! -d "$topicFolder/$name" ];then
    echo "[D]($topic/evaluation/none.md)"> "$scoreFile"
    exit 0
fi

cd $BuildDir
INSTALL_DIR=$BuildDir/installFolder
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR -DCMAKE_BUILD_TYPE=Release $topicFolder/Matcher -DUSER_NAME=$name
make
cd $Call_Path

if [ ! -f "$BuildDir/Matcher" ];then
    echo "[C]($topic/evaluation/compile_failed.md)"> "$scoreFile"
    exit 0
fi

$BuildDir/Matcher conf=$topicFolder/Matcher/Default.cfg -Name=$name ScoreFile=$scoreFile


if [ ! -f "$scoreFile" ];then
    echo "[C]($topic/evaluation/run_failed.md)"> "$scoreFile"
    exit 0
fi
