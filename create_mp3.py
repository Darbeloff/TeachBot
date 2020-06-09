#! /usr/bin/env python3
from gtts import gTTS 
tts = gTTS('Please stand back so I can reset my position.')
tts.save('safety4.mp3')