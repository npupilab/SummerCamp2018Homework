## regexp tips

^                       - begining of the string
$                       - end of the string

()                      - group 找到的匹配
[ ]*                    - 任意个数的空格
.*                      - 任意长度的内容
[^ ]                    - 表示非空格的字符
[0-9]{11}               - 表示有11个字符的数字

${BASH_REMATCH[1]}      - 第一个匹配 （从下标1开始）



## regexp for bash

regexp_course="\[(.*/.*)\]"
#regexp_student="\| *([^ ]*) * \| *[0-9]{11}.*"
regexp_student="\|[ ]*([^ ]*)[ ]* \|[ ]*[0-9]{11}.*"
    


[[ "[tool/linux]: ./tool/linux/README.md" =~ (.*) ]] && echo ${BASH_REMATCH[1]}

[[ "[tool/linux]: ./tool/linux/README.md" =~ \[(.*)\] ]] && echo ${BASH_REMATCH[1]}

[[ "[tool/linux]: ./tool/linux/README.md" =~ \[(.*/.*)\] ]] && echo ${BASH_REMATCH[1]}


regexp="\| *([^ ]*) * \| *[0-9]{11}.*"
[[ "| 张三      | 15339027461  | zd5945@126.com  | https://github.com/zdzhaoyong|" =~ $regexp ]] && echo ${BASH_REMATCH[1]}

[[ "| 张三      | 15339027461  | zd5945@126.com  | https://github.com/zdzhaoyong|" =~ \| *([^ ]*) * \|.* ]] && echo ${BASH_REMATCH[1]}



if [[ "compressed.gz" =~ ^(.*)(\.[a-z]{1,5})$ ]]; 
then 
  echo ${BASH_REMATCH[2]} ; 
else 
  echo "Not proper format"; 
fi


temp="eu-west                       140.243.64.99            "
regexp="(?:\d{1,3}\.)+(?:\d{1,3})"
if [[ $temp =~ $regexp ]]; then
  echo "found a match"
else
  echo "No IP address returned"
fi
