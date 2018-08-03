wget https://api.github.com/repos/npupilab/SummerCamp2018Homework/forks

reg='"full_name": "([^@]+)"'
 while read line
    do  
    if [[ $line =~ $reg ]];then
      Names="$Names ${BASH_REMATCH[1]}"
    fi
  done < forks

rm forks

for name in $Names;do 
echo Testing https://github.com/$name
done
