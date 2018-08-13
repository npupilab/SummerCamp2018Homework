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

if [ ! -f "$topicFolder/$name/CMakeLists.txt" ];then
    echo "[D]($topic/evaluation/none.md)"> "$scoreFile"
    exit 0
fi

cd $BuildDir
INSTALL_DIR=$BuildDir/installFolder
cmake -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR $topicFolder/$name
make install
cd $Call_Path

if [ ! -f "$INSTALL_DIR/bin/AppDemo" ];then
    echo "[C]($topic/evaluation/no_appdemo.md)"> "$scoreFile"
    exit 0
fi


if [ ! -f "$INSTALL_DIR/bin/HelloDemo" ];then
    echo "[C]($topic/evaluation/no_hellodemo.md)"> "$scoreFile"
    exit 0
fi

if [ ! -f "$INSTALL_DIR/lib/libSharedLibDemo.so" ];then
    echo "[B]($topic/evaluation/no_sharedlib.md)"> "$scoreFile"
    exit 0
fi

if [ ! -f "$INSTALL_DIR/lib/libStaticLibDemo.a" ];then
    echo "[B]($topic/evaluation/no_staticdemo.md)"> "$scoreFile"
    exit 0
fi

echo "[S]($topic/$name/CMakeLists.txt)"> "$scoreFile"

