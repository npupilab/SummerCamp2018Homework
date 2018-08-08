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
  
  reg="\|[ ]*([^ ]+)[ ]*\|[ ]([0-9]+)"
  path="\[([a-zA-Z0-9/]+)\]:[ ]([\.a-zA-Z0-9/]+)"

  name_index=0
  path_index=0

while read line
do
  if [[ $line =~ $path ]];then
    Homework[$path_index]=${BASH_REMATCH[1]}
    path_index=$path_index+1

  elif [[ $line =~ $reg ]];then
    Name[$name_index]=${BASH_REMATCH[1]}
    name_index=$name_index+1
  fi
done < $INPUTFILE

top_line="| Topic |"
second_line="| :---: |"

for((i=0; i<name_index; ++i))
do
  top_line="$top_line ${Name[i]} |"
  second_line="$second_line :--: |"
done

echo $top_line > $OUTPUTFILE
echo $second_line >> $OUTPUTFILE
  
for((i=0; i<path_index; ++i))
do
  line="| ${Homework[i]} |"
  for ((j=0; j<name_index; ++j))
  do
    if [ -f "$1/${Homework[i]}/${Name[j]}/README.md" ];then
      score=$(cat $1/${Homework[i]}/${Name[j]}/README.md)
      line="$line $score |"
    else
      line="$line D |"
    fi
  done
  echo $line >> $OUTPUTFILE
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


