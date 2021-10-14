#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    if len(sys.argv) < 2:
        print("Usage: heatmapview.py <input heatmap CSV file> [<output PNG file>]")
        exit(-1)

    # Open the CSV
    with open(sys.argv[1], "r") as fp:
        lines = fp.readlines()

    # Parse data
    dy = len(lines)
    dx = lines[0].count(",") + 1

    data = np.empty((dy, dx), dtype=np.uint32)
    for y, line in enumerate(lines):
        line = line.strip().split(",")
        data[y, :] = np.array([int(v) for v in line])

    # Plot the data
    fig = plt.figure()
    plt.imshow(data, interpolation="none", aspect="equal", cmap="jet", vmin=0, vmax=20)

    # Save to image or display
    if len(sys.argv) >= 3:
        plt.savefig(sys.argv[2], bbox_inches='tight')
    else:
        plt.show()
