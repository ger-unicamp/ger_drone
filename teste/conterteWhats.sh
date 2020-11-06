#!/bin/bash

ffmpeg -i result.mkv -c:v libx264 -profile:v baseline -level 3.0 -pix_fmt yuv420p result.mp4
