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
from dream_machine.msg import MusicCommand

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music/'

# volume ranges from 0.0 to 1.0
INITIAL_VOLUME = 0.50
VOLUME_CHANGE = 0.25

class MusicPlayer:
    def __init__(self):
        rospy.init_node('music_player', anonymous=True)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)

        # for the events system
        pygame.init()
        # for the mixer
        pygame.mixer.init()

        music.set_volume(INITIAL_VOLUME)

        self.loaded = False

    def load_song(self, song):
        music.load(MUSIC_ROOT + song)
        self.loaded = True

    def command_callback(self, message):
        def change_volume(delta):
            music.set_volume(music.get_volume() + delta)

        command = message.command
        args = message.args

        if command == MusicCommand.PLAY:
            if self.loaded:
                music.play()
        elif command == MusicCommand.STOP:
            music.stop()
        elif command == MusicCommand.PAUSE:
            music.pause()
        elif command == MusicCommand.UNPAUSE:
            music.unpause()
        elif command == MusicCommand.VOLUME_UP:
            change_volume(VOLUME_CHANGE)
        elif command == MusicCommand.VOLUME_DOWN:
            change_volume(-VOLUME_CHANGE)

        # commands with args
        elif command == MusicCommand.LOAD:
            self.load_song(args[0])

        else:
            print 'command not found:', command

if __name__ == '__main__':
    try:
        MusicPlayer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
