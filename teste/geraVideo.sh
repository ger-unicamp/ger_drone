#!/bin/bash

ffmpeg -r 10 -i result/%d.png -qscale 3 -s 752x480 result.mkv