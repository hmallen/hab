#make sure a process is always running.

#export DISPLAY=:0 #needed if you are running a simple gui app.

process=python
makerun="/home/pi/icarus_one/hab_camcontrol.py"

if ps ax | grep -v grep | grep $process > /dev/null
then
    #echo $process
    exit
else
    #echo $makerun
    $makerun &
fi

exit
