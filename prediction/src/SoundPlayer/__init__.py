import os
import time

import pygame
from typing import Dict

pygame.mixer.init(16000)


class SoundPlayer(object):
    """
    Soundplayer converts text to speech for the numbers 0 to 9
    """
    numbers = {0: 'zero', 1: 'one', 2: 'two', 3: 'three', 4: 'four', 5: 'five', 6: 'six', 7: 'seven', 8: 'eight',
               9: 'nine'
               }

    def __init__(self, resources_folder='.'):
        self.sounds = {}  # type: Dict[int, _Sound]
        for number, number_str in self.numbers.items():
            self.sounds[number] = _Sound(os.path.join(resources_folder, number_str + '.wav'))

    def playSound(self, number):
        if number in self.sounds:
            self.sounds[number].play()
        else:
            raise Exception("this number is not in the dataset")


class _Sound(object):
    def __init__(self, path):
        # type: (str) -> None
        self._snd = pygame.mixer.Sound(path)

    def play(self):
        self._snd.play()
        time.sleep(self._snd.get_length())
