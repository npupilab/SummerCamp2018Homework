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

##################### other functions below ######################

score()
{
  topicFolder=$1
  name=$2
  Function_Path=$topicFolder/evaluation
  Function_Name=evaluate
  evaluateScript=$Function_Path/evaluate.sh

  if [ -f "$evaluateScript" ];then
    . $evaluateScript $name
  else
    SCORE="D"
  fi
}

evaluate() ## do evaluation
{
  if [ -z "$1" ];then
    echo "Please input folder to evaluate. use './Statistics.sh evaluate <folder>'"
    return
	fi

  echo "Evaluate folder $1 to $Call_Path"
  INPUTFILE=$1/README.md
  OUTPUTFILE=$Call_Path/README.md

  TopicReg='\|[ ]*\[([^ ]+)\]'
  NameReg='\|[ ]*([^ ]+)[ ]*\|[ ]*[0-9]+'
  while read line
    do  
    if [[ $line =~ $TopicReg ]];then
        EXISTED=
        for topic in $Topics;do
          if [ "${BASH_REMATCH[1]}" = "$topic" ];then EXISTED=YES;fi
        done
        if [ -z "$EXISTED" ];then
          Topics="$Topics ${BASH_REMATCH[1]}"
        fi
    elif [[ $line =~ $NameReg ]];then
      Names="$Names ${BASH_REMATCH[1]}"
    fi
  done < $INPUTFILE
  
  TOPLINE="| Topic |"
  SECONDLINE="| :---: |"
  for name in $Names;do
    TOPLINE="$TOPLINE $name |"
    SECONDLINE="$SECONDLINE :---:|"
  done
  
  #cat  $INPUTFILE > $Here_Path/README.md
  rm -f $OUTPUTFILE
  while read line
    do  
    if [[ $line =~ '3. Statistics' ]];then
      echo $line >>$OUTPUTFILE
      break
    else 
      echo $line >>$OUTPUTFILE
    fi
  done < $INPUTFILE

  echo $TOPLINE >> $OUTPUTFILE
  echo $SECONDLINE >>$OUTPUTFILE

  for topic in $Topics;do
    LINE="| $topic |"

    for name in $Names;do
      score $1/$topic $name
      LINE="$LINE $SCORE |"
    done

    echo $LINE >>$OUTPUTFILE
  done

  # Compare two files
  difference="$(diff -q $INPUTFILE $OUTPUTFILE)"
  echo "Difference of $INPUTFILE $OUTPUTFILE: $difference"
  if [ -n "$difference" ]; then
    cp $OUTPUTFILE $INPUTFILE
    git commit -am "auto updated statistics"
    git push origin master
  fi
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


