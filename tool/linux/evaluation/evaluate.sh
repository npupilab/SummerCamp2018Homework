#!/bin/bash
##BEGIN_INTRODUCTION##	This function is used to stastic homework quality.
##END_INTRODUCTION##

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

echo_introduction() #Introduce myself
{
	if [ -n "$1" ];then
	FILE_Content=$(cat < $1)
	else
	FILE_Content=$(cat < $Here_Path/$File_Name)
	fi
	INTRO=${FILE_Content#*##BEGIN_INTRODUCTION##};
	INTRO=${INTRO%%##END_INTRODUCTION##*};
	echo "$INTRO"
}

echo_help() #Echo my help or others'
{
	if [ -n "$1" ];then
		FILE_Content=$(cat < $1)
	else
		FILE_Content=$(cat < $Here_Path/$File_Name)
	fi
	HELP=${FILE_Content##*##BEGIN_HELP##};
	HELP=${HELP%##END_HELP##*};
	echo "usage:  $Function_Name [options values]"
	echo options:"$HELP"
}

echo evaluating topic $topic of $name

if [ -f "$topicFolder/$name/README.md" ];then
    SCORE="[A]($topic/$name/README.md)"
  echo $SCORE
else
    SCORE="D"
fi

