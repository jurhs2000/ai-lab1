import imageManipulator as im
import matplotlib.pyplot as plt
import numpy as np

img = im.read_image("input/input1.jpg")
img = im.reduce_image(img)

# Show image with matplotlib
plt.imshow(img)
plt.show()

# Getting inital data of image
startPoint = im.get_point(img, im.PIXEL['START'])
endPoints = im.get_point(img, im.PIXEL['END'])

# Create 50x50 test matrix with some pixels set to 0 and others set to 1
path = np.random.randint(2, size=(50, 50))
img = im.replace_image(img, path)

# Show result with path
plt.imshow(img)
plt.show()
im.write_image(img)
