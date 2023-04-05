#!/bin/bash

echo "Clearing Wifi Ports"

PORT=8080

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8081

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi
PORT=8082

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8083

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8092

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

echo "Clearing Internal Ports"

PORT=8088

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8089

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8090

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8091

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

PORT=8094

PID=$(lsof -t -i:$PORT)
if [ -z "$PID"]; then
    echo "No process is running on port $PORT"
else
    echo "Killing process $PID running on port $PORT"
    sudo kill -9 "$PID"
fi

exit 0