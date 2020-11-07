#!/bin/bash

ffmpeg -r 30 -i result/%d.png -qscale 3 -s 752x480 result.mkv