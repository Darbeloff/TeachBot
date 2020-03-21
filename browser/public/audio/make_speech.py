#! /usr/bin/env python3

# Imports
import argparse, os, shutil
#import gtts
from gtts import gTTS

# Parse user input.
parser = argparse.ArgumentParser(description='Synthesize text file into audio files of speech.')
parser.add_argument('module_num', metavar='txt', type=str, help='The module number.')
args = parser.parse_args()

# Define the source and destination folders.
srcDir = '../text/module' + args.module_num + '/'
destDir = 'module' + args.module_num + '/'

# Ensure destination folder exists.
if not os.path.exists(destDir):
    os.mkdir(destDir)

# For every section text file in the source directory,
srcDirList = os.listdir(srcDir)
for textFile in srcDirList:

	# Ensure file is a text file.
	assert textFile.endswith('.txt'), 'All files in ' + srcDir + ' must be txt files. ' + textFile + ' is not.'

	# Get name of section.
	sectionTitle = textFile[0:-4]

	print('Making audio for ' + sectionTitle)

	# Path to new section's audio files. If one exists already, clear it.
	linePath = destDir + sectionTitle + '/'
	if os.path.exists(linePath):
		shutil.rmtree(linePath)
	os.mkdir(linePath)

	# For each line in section text file,
	textFile = open(srcDir + textFile)
	lines = textFile.read().split('\n')
	for x in range(len(lines)):

		# Synthesize text into audio file and save.
		tts = gTTS(text = lines[x], lang='en')
		try:
			tts.save(linePath + 'line' + str(x) +'.mp3')
		except:
			print('Unable to generate mp3 for the following:\n\t' + lines[x] + 'Ensure the line contains no unpronounceable characters.')