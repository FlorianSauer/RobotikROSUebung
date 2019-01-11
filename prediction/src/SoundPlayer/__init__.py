import os

import pygame


class SoundPlayer(object):
    """
    Soundplayer converts text to speech for the numbers 0 to 9
    """

    def __init__(self, resources_folder='.'):

        pygame.mixer.init(16000)
        self.snd0 = pygame.mixer.Sound(os.path.join(resources_folder, 'zero.wav'))
        self.snd1 = pygame.mixer.Sound(os.path.join(resources_folder, 'one.wav'))
        self.snd2 = pygame.mixer.Sound(os.path.join(resources_folder, 'two.wav'))
        self.snd3 = pygame.mixer.Sound(os.path.join(resources_folder, 'three.wav'))
        self.snd4 = pygame.mixer.Sound(os.path.join(resources_folder, 'four.wav'))
        self.snd5 = pygame.mixer.Sound(os.path.join(resources_folder, 'five.wav'))
        self.snd6 = pygame.mixer.Sound(os.path.join(resources_folder, 'six.wav'))
        self.snd7 = pygame.mixer.Sound(os.path.join(resources_folder, 'seven.wav'))
        self.snd8 = pygame.mixer.Sound(os.path.join(resources_folder, 'eight.wav'))
        self.snd9 = pygame.mixer.Sound(os.path.join(resources_folder, 'nine.wav'))

    def playSound(self, argument):
        if argument == 0:
            self.snd0.play()
        elif argument == 1:
            self.snd1.play()
        elif argument == 2:
            self.snd2.play()
        elif argument == 3:
            self.snd3.play()
        elif argument == 4:
            self.snd4.play()
        elif argument == 5:
            self.snd5.play()
        elif argument == 6:
            self.snd6.play()
        elif argument == 7:
            self.snd7.play()
        elif argument == 8:
            self.snd8.play()
        elif argument == 9:
            self.snd9.play()
        else:
            raise Exception("this number is not in the dataset")
