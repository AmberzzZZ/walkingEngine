#!/bin/bash
#
# This Script compile and copyfiles through the command line
# instead of Xcode
#

COMPONENT1='SimRobot'
COMPONENT2='Nao'
CONFIG='Develop'        # one can also use Release/Debug

echo "******** FUNCTION LIST ***********"
echo "******** 1.MAKE        ***********"
echo "******** 2.COPYFILES   ***********"
echo "88888888 22.COPYFILES  88888888888"
echo "******** 3.LOGIN       ***********"
echo "88888888 33.LOGIN      88888888888"
echo "******** 4.SIMROBOT    ***********"

read number

if [ $number -eq 1 ];then
cd Make/macOS
./compile $COMPONENT1 $CONFIG
./compile $COMPONENT2 $CONFIG
cd;

elif [ $number -eq 2 ];then
cd Make/macOS
echo "Input your IP:192.168.20."
read ip
./copyfiles $CONFIG 192.168.20.$ip -r
cd

elif [ $number -eq 22 ];then
cd Make/macOS
echo "Input your IP:10.0.20."
read ip
./copyfiles $CONFIG 10.0.20.$ip -r
cd

elif [ $number -eq 3 ];then
cd Make/macOS
echo "Input your IP:192.168.20."
read ip
./login 192.168.20.$ip 
cd

elif [ $number -eq 33 ];then
cd Make/macOS
echo "Input your IP:10.0.20."
read ip
./login 10.0.20.$ip 
cd

elif [ $number -eq 4 ];then
cd Build/macOS/SimRobot/Develop/SimRobot.app/Contents/MacOS
./SimRobot;

fi






