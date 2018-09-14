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
	File_Name=$Function_Name.func
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

##################### other functions below ######################

evaluate() ## do evaluation
{
  if [ -z "$1" ];then
    echo "Please input folder to evaluate. use './Statistics.sh evaluate <folder>'"
    return
	fi

  echo "Evaluate folder $1"
  INPUTFILE=$1/README.md
  OUTPUTFILE=$Here_Path/README.md
  
# Please add your implementation here
	TitleReg='\|[ ]*\[([^ ]*)\]'
	NameReg='\|[ ]*([^ ]*)[ ]*\|[ ]*[1-9]'
	Titles=;Names=;
	while read line; do
		if [[ $line =~ $TitleReg ]]; then
			Titles="$Titles ${BASH_REMATCH[1]}"
		elif [[ $line =~ $NameReg ]]; then
			Names="$Names ${BASH_REMATCH[1]}"
		fi
	done < $INPUTFILE

  TOPLINE="| Topic |"
  SECONDLINE="| :---: |"
  for name in $Names;do
    TOPLINE="$TOPLINE $name |"
    SECONDLINE="$SECONDLINE :---:|"
  done
  echo $TOPLINE > $OUTPUTFILE
  echo $SECONDLINE >>$OUTPUTFILE
  for title in $Titles; do
  	LINE="| $title |"
  	for name in $Names; do
  		if [[ -a "$1/$title/$name/README.md" ]]; then
  			SCORE=$(cat $1/$title/$name/README.md)
  			LINE="$LINE $SCORE |"
  		else
  			LINE="$LINE D |"
  		fi
  	done
  	echo $LINE>>$OUTPUTFILE
  done
}

######################  main below  ##############################
if [ -n "$1" ];then
	while [ -n "$1" ]; do
	case $1 in
##BEGIN_HELP##
		-h)     shift 1;echo_help;exit 1;;                   #Show usages 
		-i)     shift 1;echo_introduction;exit 1;;           #Show introduction 
		-edit)  shift 1;gedit $Here_Path/$File_Name;exit 1;; #Edit this function 
		-e)     shift 1;evaluate $*;exit 1;; #Evaluate
		-*)     echo "error: no such option $1. -h for help";exit 1;; 
		*)      $*;exit 1;;                                  #Call function here
##END_HELP##
	esac
	done
else
	echo_help
fi
#echo ---------------------End Of $Function_Name-----------------------


