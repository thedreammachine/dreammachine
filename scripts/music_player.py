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
from pygame.mixer import music
import os
import Queue

from std_msgs.msg import String
from dream_machine.msg import MusicCommand, MusicState

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music/'

# volume ranges from 0.0 to 1.0
INITIAL_VOLUME = 0.50
VOLUME_CHANGE = 0.25

class MusicPlayer:
    def __init__(self):
        rospy.init_node('music_player', anonymous=True)
        self.r = rospy.Rate(2.0)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)
        self.music_player_state_pub = rospy.Publisher('~music_player_state', MusicState, queue_size=10)
        # for the events system
        pygame.init()
        # for the mixer
        pygame.mixer.init()

        music.set_volume(INITIAL_VOLUME)

        self.loaded = False
        self.paused = False

        while not rospy.is_shutdown():
            self.loop()
            self.r.sleep()

    def loop(self):
        state = MusicState(
            music.get_busy(), # check if player is playing
            music.get_pos() / 1000, # current_time (api doesn't support)
            300, # total length of the song (api doesn't support)
            music.get_volume(),
            "artist",
            "title"
        )
        self.music_player_state_pub(state)

    def load_song(self, song):
        music.load(MUSIC_ROOT + song)
        self.loaded = True

    def seek_to(self, seekTime):
        if self.loaded:
            music.play(0, float(args[0]))

    def command_callback(self, message):
        def change_volume(delta):
            music.set_volume(music.get_volume() + delta)

        command = message.command
        args = message.args

        if command == MusicCommand.PLAY:
            if self.loaded && !self.paused:
                music.play()
            else:
                music.unpause()
                self.paused = False
        elif command == MusicCommand.STOP:
            music.stop()
        elif command == MusicCommand.PAUSE:
            music.pause()
            self.paused = True
        elif command == MusicCommand.UNPAUSE:
            music.unpause()
            self.paused = False
        elif command == MusicCommand.VOLUME_UP:
            change_volume(VOLUME_CHANGE)
        elif command == MusicCommand.VOLUME_DOWN:
            change_volume(-VOLUME_CHANGE)


        # commands with args
        elif command == MusicCommand.LOAD:
            self.load_song(args[0])
        elif command == MusicCommand.SEEK_TO:
            self.seek_to(args[0])
        elif command == MusicCommand.SET_VOLUME
            music.set_volume(float(args[0]))
        else:
            print 'command not found:', command

if __name__ == '__main__':
    try:
        MusicPlayer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
