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

from std_msgs.msg import String
from dream_machine.msg import MusicCommand

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music/'

MUSIC_FILE = 'rick-roll.wav'

# volume ranges from 0.0 to 1.0
INITIAL_VOLUME = 0.50
VOLUME_CHANGE = 0.25

# TODO support loading other files, looping, queuing.
class MusicPlayer:
    def __init__(self):
        rospy.init_node('music_player', anonymous=True)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)
        pygame.mixer.init()
        music.load(MUSIC_ROOT + MUSIC_FILE)
        music.set_volume(INITIAL_VOLUME)

    def command_callback(self, command):
        def change_volume(delta):
            music.set_volume(music.get_volume() + delta)

        command = command.command

        if command == MusicCommand.PLAY:
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
        else:
            print 'command not found:', command

if __name__ == '__main__':
    try:
        MusicPlayer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
