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

string command
string[] args
