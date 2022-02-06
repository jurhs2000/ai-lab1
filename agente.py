from graphsearch import BFS
import imageManipulator as im
import matplotlib.pyplot as plt
import numpy as np
from problem import Problem

img = im.read_image("input/input3.png")
img, startpoints, endpoints = im.reduce_image(img)
print(startpoints, endpoints)

# Show image with matplotlib
plt.imshow(img)
plt.show()

problem = Problem(img, startpoints, endpoints)
algorithm = BFS(problem)
result = algorithm.execute()

img = im.replace_image(img, result)

# Show result with path
plt.imshow(img)
plt.show()
im.write_image(img)
