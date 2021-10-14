#!/bin/bash

# Invoke nextpnr with router2 (the default for nexus) and add the option
# "--router2-heatmap heatmap" to it. This will write a series of
# "heatmap-congestion_*.csv" files for each router iteration.
#
# Once done put those files in the same folder as this script and
# heatmapview.py Make sure that you have ffmpeg installed. Then run the script

set -e

# First convert all dumped CSVs to images
echo "Converting heatmaps..."
for CSV_FILE in `find . -name "heatmap-congestion_*.csv"`; do
    echo ${CSV_FILE}
    python3 heatmapview.py ${CSV_FILE} ${CSV_FILE/.csv/.png}
done

# Make a slideshow video using ffmpeg
ffmpeg -y -framerate 2 -i heatmap-congestion_%d.png -c:v mjpeg -q:v 2 heatmap-congestion.mp4
