#!/bin/sh
#echo $$ > ~/videostreamer.pid

# check if ffserver is already running (just in case)
if [ -z `pidof ffserver` ]
  then ffserver -f ~/px_ffserver.conf &
fi

# check if feed target file already exists and remove if necessary
#if []

# feed the image stream to ffmpeg and send it to the ffserver
#px_videoprepare | ffmpeg -r 15 -s 640x480 -f rawvideo -i pipe: http://localhost:8090/feed.ffm
px_videoprepare | ffmpeg -r 30 -s 640x480 -f rawvideo -i pipe: udp://192.168.0.2:1337/live.mpg


#px_videoprepare | ffmpeg -r 15 -s 640x480 -f rawvideo -i pipe: /home/pixhawk/feed.avi
#px_videoprepare | ffmpeg -r 15 -s 640x480 -f rawvideo -i pipe: /tmp/feed.ffm
#px_videoprepare | ffmpeg -r 30 -s 640x480 -f rawvideo -i pipe: /var/www/feed.mpg




#px_videoprepare | ffmpeg -r 30 -s 320x240 -f rawvideo -i pipe: -f audio_device -i /dev/dsp /tmp/feed.ffm
#px_videoprepare | ffmpeg -r 30 -s 320x240 -f rawvideo -i pipe: http://localhost:8090/feed.ffm

#px_video | ffmpeg -r 30 -s 320x240 -f rawvideo -i pipe: -f audio_device -i /dev/dsp http://localhost:8090/feed.ffm
#http://www.rmatsumoto.org/camera/dc1394-ohphone-old.html

exit
