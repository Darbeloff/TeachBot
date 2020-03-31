#! /usr/bin/env python3
from gtts import gTTS 
tts = gTTS('Oops. My arm is stuck. Can you help me? Pull my gripper out over the table as far as you can.')
tts.save('safety3.mp3')