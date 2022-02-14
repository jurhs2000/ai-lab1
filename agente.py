from graphsearch import BFS, a_star
import imageManipulator as im
import matplotlib.pyplot as plt
import numpy as np
from problem import Problem

img = im.read_image("input/input4.bmp")
img, startpoints, endpoints = im.reduce_image(img)
print(startpoints, endpoints)

# Show image with matplotlib
plt.imshow(img)
plt.show()

problem = Problem(img, startpoints, endpoints)
option = 0
while option != 4:
  print("1. BFS")
  print("2. DFS")
  print("3. A*")
  print("4. Exit")
  option = int(input("Choose an option: "))
  if option == 1:
    algorithm = BFS(problem)
    path, explored, frontier = algorithm.execute()
    option = 4
  elif option == 2:
    pass
  elif option == 3:
    algorithm = a_star(problem)
    path, explored, frontier = algorithm.execute()
    option = 4
  elif option == 4:
    break
  else:
    print("Invalid option")

img = im.replace_image(img, path, explored, frontier)

# Show result with path
plt.imshow(img)
plt.show()
im.write_image(img)
