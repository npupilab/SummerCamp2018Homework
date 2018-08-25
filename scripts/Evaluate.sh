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

  SCORE="D"
  SCOREFILE=$topic/$name/Score.txt
  mkdir -p $topic/$name
  if [ -f "$evaluateScript" ];then
    bash $evaluateScript $topic $name $SCOREFILE
    #echo "$evaluateScript $topic $name $SCOREFILE"
  elif [ -f "$Function_Path/evaluate.py" ];then
    python $Function_Path/evaluate.py $topic $name $SCOREFILE
    #echo "pyout: $pout"
  fi
  
  if [ -f "$SCOREFILE" ];then
    SCORE="$(cat $SCOREFILE)"
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

  TopicReg='\[([a-zA-Z0-9/]+)\]:[ ]([\.a-zA-Z0-9/]+)'
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


  if [ -n "$2" ];then
    Topics=$2
  fi

  if [ -n "$3" ];then
    Names=$3
  fi
  
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

  echo $TOPLINE
  echo $TOPLINE >> $OUTPUTFILE
  echo $SECONDLINE >>$OUTPUTFILE

  for topic in $Topics;do
    LINE="| $topic |"

    for name in $Names;do
      score $1/$topic $name
      LINE="$LINE $SCORE |"
    done

    echo $LINE >>$OUTPUTFILE
    echo $LINE
  done

  # Compare two files
  if [ -n "$CommitUpdate" ];then
    difference="$(diff -q $INPUTFILE $OUTPUTFILE)"
    echo "Difference of $INPUTFILE $OUTPUTFILE: $difference"
    if [ -n "$difference" ]; then
      cp $OUTPUTFILE $INPUTFILE
      cat $INPUTFILE
      git add $Here_Path/../README.md
      git commit -m "auto updated statistics"
      git push origin master
    fi
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
		-e)     shift 1;evaluate $*;exit 0;; #Evaluate
		-commit) shift 1;CommitUpdate=YES;; #Evaluate
		-*)     echo "error: no such option $1. -h for help";exit 1;; 
		*)      shift 1;exit 1;;                                  #Call function here
##END_HELP##
	esac
	done
else
	echo_help
fi
#echo ---------------------End Of $Function_Name-----------------------


