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

topicFolder=$1
topic=$2
name=$3

if [ ! -f "$topicFolder/$name/README.md" ];then
    export SCORE="D"
    exit 0
fi

if [ ! -f "$topicFolder/$name/Statistics.sh" ];then
    export SCORE="D"
    exit 0
fi

testFolder=$Call_Path/tool/linux/$name
mkdir -p $testFolder
cp -f $topicFolder/$name/Statistics.sh $testFolder/Statistics.sh
rm -f $testFolder/README.md
output="$(bash $testFolder/Statistics.sh -e $topicFolder)"

if [ ! -f "$testFolder/README.md" ];then # Failed to output README.md
  echo "Failed to output $testFolder/README.md"
  export SCORE="C"
  exit 0
fi


difference="$(diff -q $testFolder/README.md $topicFolder/$name/README.md)"
if [ -n "$difference" ]; then
  echo "File $topicFolder/$name/README.md are not created by $testFolder/Statistics.sh, diff: \"$difference\""
#  export SCORE="C";exit 0
fi

# Check output correct or not

  INPUTFILE=$topicFolder/README.md
  OUTPUTFILE=$testFolder/DEMO.txt

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
  
  TOPLINE="\| *Topic *|"
  for name in $Names;do
    TOPLINE="$TOPLINE *$name *\|"
  done

checkline()
{
  topic="${Topics[$1 - 2]}"
  LINE="\| *$topic *|"
  for name in $Names;do
    if [ -f "$1/$topic/$name/README.md" ];then
        CONTENT=$(cat $1/$topic/$name/README.md)
        LINE="$LINE *$CONTENT *| *"
      else
        LINE="$LINE *D *| *"
      fi
  done
  echo $LINE
}

lineID=0;
maxLineSize=2
for topic in $Topics;do maxLineSize=$(expr $maxLineSize + 1);done

while read line ; do
  if [ $lineID -ge $maxLineSize ]; then
    echo "lineID $lineID ($line) should not greater than $maxLineSize"
  #  export SCORE="B";exit 0
  fi
  reg=""
  case "$lineID" in
    "0") reg="$TOPLINE";; 
    "1") reg="";;
    "*") reg="$(checkline $lineID $line)";;
  esac
  lineID=$(expr $lineID + 1)
  if [[ ! $line =~ $reg ]];then
    echo "$line not match $reg"
    export SCORE="B";exit 0
  fi
done < $testFolder/README.md

export SCORE="[S]($topic/$name/README.md)"
