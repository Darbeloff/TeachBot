#! /usr/bin/env python3
from gtts import gTTS 
tts = gTTS('To avoid my arm colliding with myself, I cannot move to this waypoint.')
tts.save('collision.mp3')