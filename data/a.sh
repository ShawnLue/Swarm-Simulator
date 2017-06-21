#!/bin/bash

ffmpeg -r 1/15 -f image2 -i ./snap%d.pdf -s 1000*1000 -y simulation.avi
