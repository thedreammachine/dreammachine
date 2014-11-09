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

MUSIC_FILE = 'rick-roll.wav'

# volume ranges from 0.0 to 1.0
INITIAL_VOLUME = 0.50
VOLUME_CHANGE = 0.25

# TODO support loading other files, looping, queuing.
class MusicPlayer:
    def __init__(self):
        rospy.init_node('music_player', anonymous=True)

        rospy.Subscriber("/music_commands", MusicCommand, self.command_callback)

        # for the events system
        pygame.init()
        # for the mixer
        pygame.mixer.init()

        self.event_id = pygame.USEREVENT
        self.song_queue = Queue.Queue()

        music.set_endevent(self.event_id)
        self.current_song = MUSIC_FILE
        music.set_volume(INITIAL_VOLUME)

        r = rospy.Rate(10)
        while not rospy.is_shutdown():
          self.loop()
          r.sleep()

    def load_song(self, song):
        music.load(MUSIC_ROOT + song)

    def command_callback(self, message):
        def change_volume(delta):
            music.set_volume(music.get_volume() + delta)

        command = message.command
        args = message.args

        if command == MusicCommand.PLAY:
            if self.current_song:
                self.load_song(self.current_song)
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
        elif command == MusicCommand.NEXT_SONG:
            self.next_song()

        # commands with args
        elif command == MusicCommand.QUEUE:
            if not self.current_song:
                self.current_song = args[0]
            else:
                self.song_queue.put_nowait(args[0])

        else:
            print 'command not found:', command

    def next_song(self):
        try:
          next_song = self.song_queue.get_nowait()
          self.current_song = next_song
          self.load_song(next_song)
          music.play()
        except Queue.Empty:
          self.current_song = None
          music.stop()

    def loop(self):
        for event in pygame.event.get():
            if event.type == self.event_id:
                self.next_song()

if __name__ == '__main__':
    try:
        MusicPlayer()
    except rospy.ROSInterruptException: pass
