#!/bin/sh

#PID=$(ps ax |grep rostopic pub |grep -v grep |awk '{print $1}')
#ps ax |grep "rostopic pub" |grep -v grep |awk '{print $1}'
#echo xx $PID
#kill $PID #2>/dev/null


kill $(ps ax |grep "rostopic pub" |grep -v grep |awk '{print $1}') 2>/dev/null
