#!/usr/bin/python
# Convert gif files to  base64-encoded python files. This solves a .gif "no such file" runtime error.

import base64

with open("led.py","w") as f:
   f.write('GREEN="""%s"""\n' % base64.b64encode(open("green_led.gif","rb").read()))
   f.write('RED="""%s"""\n' % base64.b64encode(open("red_led.gif","rb").read()))
   f.write('ORANGE="""%s"""\n' % base64.b64encode(open("orange_led.gif","rb").read()))
   f.write('ICON="""%s"""\n' % base64.b64encode(open("circle_icon.gif","rb").read()))
