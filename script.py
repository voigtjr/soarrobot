#!/usr/bin/python

from os import popen, system
command = 'grep -r april_voigt *'
p = popen(command)
files = {}
while 1:
    line = p.readline().strip()
    if not line:
        break
    if not '.java:' in line:
        continue
    files[line] = line
p.close()
for filename in files:
    print filename
