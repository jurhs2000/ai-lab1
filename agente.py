from graphsearch import BFS, a_star, DFS
import imageManipulator as im
import matplotlib.pyplot as plt
from problem import Problem

image_url = ''
option = 0
while option != 1 and option != 2 and option != 3 and option != 4:
  print('Please select the image to work with:')
  print('1. Image 1')
  print('2. Image 2')
  print('3. Image 3')
  print('4. Image 4')
  option = int(input("Choose an image: "))
  if option == 1:
    image_url = 'input/input3.png'
  elif option == 2:
    image_url = 'input/input4.bmp'
  elif option == 3:
    image_url = 'input/Prueba Lab1.bmp'
  elif option == 4:
    image_url = 'input/turing.png'

img = im.read_image(image_url)
img, startpoints, endpoints = im.reduce_image(img)

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
  option = int(input("Choose an algorithm: "))
  if option == 1:
    algorithm = BFS(problem)
    path, explored, frontier = algorithm.execute()
    option = 4
  elif option == 2:
    algorithm = DFS(problem, image_url)
    path, explored, frontier = algorithm.execute()
    option = 4
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
