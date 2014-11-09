# command constants:

# these require no arguments

string PLAY=play
string STOP=stop
string PAUSE=pause
string UNPAUSE=unpause
string VOLUME_UP=volume_up
string VOLUME_DOWN=volume_down
string NEXT_SONG=next_song

# these require one argument

# set the given audio file to play after the current one, or play next time
# 'play' is called if nothing is currently playing or queued.
#
# argument: filename
string QUEUE=queue

string command
string[] args
