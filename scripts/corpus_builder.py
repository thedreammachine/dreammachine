#!/usr/bin/python

import os

import voice_constants

SCRIPT_DIR = os.path.dirname(__file__)

MUSIC_ROOT = os.environ['HOME'] + '/dream_machine_music'

# Take the output of this script and upload it to:
# http://www.speech.cs.cmu.edu/tools/lmtool-new.html
#
# to generate the other files needed for speech recognition

class CorpusBuilder:

  def build_corpus(self):
    corpus_file = open('/home/motionlab/catkin_ws/src/dream_machine/voice/commands.corpus', 'w')

    voice_constant_names = [x for x in dir(voice_constants) if not x.startswith('_')]
    voice_constant_values = [getattr(voice_constants, x) for x in voice_constant_names]

    for constant in voice_constant_values:
      corpus_file.write(constant + '\n')

    for corpus_line in self.read_song_directory().keys():
      corpus_file.write(corpus_line + '\n')


  def read_song_directory(self):
    _, _, files = os.walk(MUSIC_ROOT).next()
    corpus_dic = {}

    for file in files:
      no_extension = file.split('.')[0]
      artist, song = [x.strip() for x in no_extension.split('-')]
      corpus_dic["play %s" % song.split()[0].lower()] = file

    return corpus_dic

if __name__ == '__main__':
  CorpusBuilder().build_corpus()
