#!/usr/bin/env python

import sys
import time
from os.path import expanduser

from matplotlib import pyplot

PATH_LOGS = '/root/ros2_ws/rrt_graph.log'

def plot_coords(ax, x, y, color, zorder, size=0.5):
    ax.plot(x, y, 'o', color=color, zorder=zorder, markersize=size)
    
def plot_line(ax, p1, p2, width=0.3):
    ax.plot(p1, p2, color='blue', linestyle='-', linewidth=width)

fig = pyplot.figure(1, dpi=90)
#pyplot.margins(x=0.1, y=0.1)
ax = fig.add_subplot(111)
ax.set_title('RRT generado')
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')

i = 0

with open(PATH_LOGS) as f:
	for line in f:
		if i < 2:
			x1, y1, theta1 = map(float, line.split())
			
			if i == 0: # Nodo start
				color = 'red'
			if i == 1: # Nodo goal
				color = 'green'
			
			plot_coords(ax, x1, y1, color, 1, 5)

		else: 
			x1, y1, theta1, x2, y2, theta2 = map(float, line.split())
			
			color = 'blue'

			plot_coords(ax, x1, y1, color, 0, 1.5)
			plot_coords(ax, x2, y2, color, 0, 1.5)
			plot_line(ax, [x1, x2],[y1, y2], 0.5)
		i = i+1
		
		
pyplot.tight_layout()
pyplot.show()
