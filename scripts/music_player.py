#!/usr/bin/python

"""
music_player.py listens to music commands (play, pause, volume down, etc.) and
plays music accordingly.

listeners:
    /music_commands (dream_machine/MusicCommand) - executes the given music
      command.
"""

import rospy
import pygame
import swmixer 
import os
import Queue

from std_msgs.msg import String
from dream_machine.msg import MusicCommand, MusicState

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music/'

# volume ranges from 0.0 to 1.0
INITIAL_VOLUME = 0.50
VOLUME_CHANGE = 0.25
SAMPLE_RATE = 44100
CHUNK_SIZE = 1024
class MusicPlayer:


    def __init__(self):
        rospy.init_node('music_player', anonymous=False)
        self.r = rospy.Rate(2.0)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)
        self.music_player_state_pub = rospy.Publisher('music_player_state', MusicState, queue_size=10)

        swmixer.init(samplerate=SAMPLE_RATE, chunksize=CHUNK_SIZE, stereo=False)
        swmixer.start()

        self.sound = None
        self.channel = None

        self.is_playing = False
        self.is_paused = False

        while not rospy.is_shutdown():
            self.loop()
            self.r.sleep()

    def loop(self):
        if self.is_song_loaded() and (self.channel.get_position() == self.sound.get_length()):
                self.on_end_song();

        state = MusicState(
            self.is_playing, # check if player is playing
            self.get_song_position(), 
            self.get_song_length(), 
            self.get_song_volume(),
            "artist",
            "title"
        )
        self.music_player_state_pub.publish(state)

    def get_song_position(self):
        if(self.is_song_loaded()):
            return int(self.channel.get_position() / SAMPLE_RATE);
        else:
            return 0

    def get_song_length(self):
        if(self.is_song_loaded()):
            return int(self.sound.get_length()  / SAMPLE_RATE)
        else:
            return 0

    def get_song_volume(self):
        if(self.is_song_loaded()):
            return float(self.channel.get_volume())
        else:
            return 0.0

    def on_end_song(self):
        # HANDLE END SONG LOGIC
        pass

    def load_song(self, song):
        if(self.is_song_loaded() and self.is_playing):
            self.channel.stop()

        self.sound = swmixer.Sound(MUSIC_ROOT + song)
        self.channel = self.sound.play()
        self.is_playing = True

    def seek_to(self, seekTime):
        if self.is_song_loaded():
            self.channel.set_position(int(seekTime) * SAMPLE_RATE)

    def is_song_loaded(self):
        return self.sound != None and self.channel != None;

    def command_callback(self, message):
        def change_volume(delta):
            self.channel.set_volume(self.channel.get_volume() + delta)

        command = message.command
        args = message.args

        print(command)

        if command == MusicCommand.PLAY:
            if (self.is_song_loaded() and not self.is_playing):
                if (self.is_paused):
                    self.channel.unpause()
                    self.is_paused = False
                    self.is_playing = True
                else:
                    self.channel.play()
                    self.is_playing = True
        elif command == MusicCommand.STOP:
            self.channel.stop()
            self.is_playing = False
        elif command == MusicCommand.PAUSE:
            if(self.is_song_loaded() and self.is_playing):
                self.channel.pause()
                self.is_playing = False
                self.is_paused = True
        elif command == MusicCommand.UNPAUSE:
            if(self.is_song_loaded() and not self.is_playing):
                self.channel.unpause()
                self.is_playing = True
                self.is_paused = False
        elif command == MusicCommand.VOLUME_UP:
            change_volume(VOLUME_CHANGE)
        elif command == MusicCommand.VOLUME_DOWN:
            change_volume(-VOLUME_CHANGE)


        # commands with args
        elif command == MusicCommand.LOAD:
            self.load_song(args[0])
        elif command == MusicCommand.SEEK_TO:
            self.seek_to(args[0])
        elif command == MusicCommand.SET_VOLUME:
            self.channel.set_volume(float(args[0]))
        else:
            print 'command not found:', command

if __name__ == '__main__':
    try:
        MusicPlayer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
