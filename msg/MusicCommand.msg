# command constants:

# these require no arguments

string PLAY=play
string STOP=stop
string PAUSE=pause
string UNPAUSE=unpause
string VOLUME_UP=volume_up
string VOLUME_DOWN=volume_down


# these require one argument

# Set the given audio file to play next time play is called
#
# argument: filename
string LOAD=load

# Seek to the given time in the audio file
#
# argument: seek_time
string SEEK_TO=seek_to

# Seek to the given time in the audio file
#
# argument: volume_val
string SET_VOLUME=set_volume

string command
string[] args
