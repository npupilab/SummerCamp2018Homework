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

read_file()
{
    local contents=""
    
    while IFS= read -r var; do
        contents="${contents}${var}"
    done < "$1"
    
    echo "$contents"
}

read_file2()
{
    local contents=$(cat < $1)
    echo "$contents"
}


##################### other functions below ######################

evaluate() ## do evaluation
{
    if [ -z "$1" ];then
        echo "Please input folder to evaluate. use './Statistics.sh evaluate <folder>'"
        return
	fi

    echo "Evaluate folder $1"
    
    DATAPATH=$1
    INPUTFILE=$1/README.md
    OUTPUTFILE=$Here_Path/README.md
  
    # Please add your implementation here
    
    course=""
    students=""

    couse_n=0
    students_n=0
    
    # read $INPUTFILE's contents
    while IFS= read -r var; do
        #echo "$var"
        
        # detect course 
        regexp="\[(.*/.*)\]"
        if [[ "$var" =~ $regexp ]]; then
            s=${BASH_REMATCH[1]}
            if [[ ! "$course" =~ $s ]]; then
                course="$course $s"
                course_n=$(($course_n + 1))
            fi
        fi
        
        # detect person, which follows phone number
        regexp="\| *([^ ]*) * \| *[0-9]{11}.*"
        if [[ "$var" =~ $regexp ]]; then
            s="${BASH_REMATCH[1]}"
            if [[ ! "$students" =~ $s ]]; then
                students="$students $s"
                students_n=$(($students_n + 1))
            fi
        fi
    done < "$INPUTFILE"
    
    echo "Course [$course_n]   : $course"
    echo "Students [$students_n] : $students"
    
    # echo header
    line="| Topic "
    for s in $students; do
        line="$line | $s"
    done
    line="$line |"
    echo $line > $OUTPUTFILE

    line="| :---:"
    for ((i=1; i<=$students_n; i++)); do
        line="$line | :---:"
    done
    line="$line |";
    echo $line >> $OUTPUTFILE

    # for each course
    for c in $course; do
        line="| [$c]"

        # for each students
        for s in $students; do
            p="${DATAPATH}/$c/$s/README.md"
            
            v="D"
            if [[ -e $p ]]; then
                v=$(read_file $p)
            fi

            line="$line | $v"
        done
        
        line="$line |"
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


