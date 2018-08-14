#!/bin/bash
here_path=$(dirname $(readlink -f $0))
INPUTFILE=$2/README.md
OUTPUTFILE=$here_path/README.md


TopicReg='\|[ ]*\[([^ ]+)\]'
NameReg='\|[ ]*([^ ]+)[ ]*\|[ ]*[0-9]+'
while read line
    do  
    if [[ $line =~ $TopicReg ]];then
        EXISTED=
        for topic in $Topics;do
          if [ "${BASH_REMATCH[1]}" == "$topic" ];then 
              EXISTED=YES;
              #break
          fi
        done
        if [ -z "$EXISTED" ];then
          Topics="$Topics ${BASH_REMATCH[1]}"
        fi
   elif [[ $line =~ $NameReg ]];then
        EXISTED=
        for name in $Names;do
          if [ "${BASH_REMATCH[1]}" == "$name" ];then 
              EXISTED=YES;
              #break
          fi
        done
        if [ -z "$EXISTED" ];then
     		Names="$Names ${BASH_REMATCH[1]}"
        fi
   fi
done < $INPUTFILE

TOPLINE="| Topic |"
SECONDLINE="| :---: |"
for name in $Names;do
    TOPLINE="$TOPLINE $name |"
    SECONDLINE="$SECONDLINE :---:|"
done

#cat  $INPUTFILE > $Here_Path/README.md
echo $TOPLINE > $OUTPUTFILE
echo $SECONDLINE >>$OUTPUTFILE

for topic in $Topics;do
    LINE="| $topic |"
    for name in $Names;do
        if [ -f "$2/$topic/$name/README.md" ];then
            SCORE=$(cat $2/$topic/$name/README.md)
            LINE="$LINE $SCORE|"
        else
            LINE="$LINE D |"
        fi
    done
    echo $LINE >>$OUTPUTFILE
done  
