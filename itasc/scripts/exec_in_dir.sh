#!/bin/bash

export PROCESS="42"


if [ "$2" == "" ]; then
    echo "Usage: $0 <dir> <cmd> [params*]"
    exit 1
fi

echo starting process

mkdir -p $1
cd $1
${@:2} &

trap "kill -n 15 $!; exit 0" 2

echo process $! started

while true
do
  sleep 100 # save cpu time...
done



