#!/usr/bin/python

import rospy
import pygame
from pygame.mixer import music
import os
from music_constants import *

from std_msgs.msg import String
from dream_machine.msg import MusicCommand

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music/'

MUSIC_FILE = 'rick-roll.wav'

# volume ranges from 0.0 to 1.0
VOLUME_CHANGE = 0.1

# TODO support loading other files, looping, queuing.
class MusicPlayer:
    def __init__(self):
        rospy.init_node('music_player', anonymous=True)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)
        pygame.mixer.init()
        music.load(MUSIC_ROOT + MUSIC_FILE)

    def command_callback(self, command):
        def change_volume(delta):
            music.set_volume(music.get_volume() + delta)

        command = command.command

        if command == PLAY:
            music.play()
        elif command == STOP:
            music.stop()
        elif command == PAUSE:
            music.pause()
        elif command == UNPAUSE:
            music.unpause()
        elif command == VOLUME_UP:
            change_volume(VOLUME_CHANGE)
        elif command == VOLUME_DOWN:
            change_volume(-VOLUME_CHANGE)
        else:
            print 'command not found:', command

if __name__ == '__main__':
    try:
        MusicPlayer()
        rospy.spin()
    except rospy.ROSInterruptException: pass
