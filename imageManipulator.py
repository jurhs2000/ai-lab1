from PIL import Image

# Matrix size to resize image
RESIZE = (50, 50)
# Threshold percentage of white pixels
THRESHOLD_PERCENTAGE = 0.9

# Pixels types
PIXEL = {
  'BLACK': (0, 0, 0),
  'WHITE': (255, 255, 255),
  'THRESHOLD': (255 * THRESHOLD_PERCENTAGE, 255 * THRESHOLD_PERCENTAGE, 255 * THRESHOLD_PERCENTAGE),
  'START': (0, 255, 0),
  'END': (255, 0, 0),
  'PATH': (0, 0, 255)
}

# Read image
def read_image(path):
  return Image.open(path)

# Write image to file
def write_image(img):
  img.save('output/output.bmp')

# Reduce all pixels of image in 50x50 matrix
def reduce_image(img):
  reduced = img.resize(RESIZE)
  # convert all pixels of Image to black if they are less than 50% white
  for i in range(reduced.size[0]):
    for j in range(reduced.size[1]):
      pixel = reduced.getpixel((i, j))
      # If pixel is less than 225
      if pixel[0] < PIXEL['THRESHOLD'][0] and pixel[1] < PIXEL['THRESHOLD'][1] and pixel[2] < PIXEL['THRESHOLD'][2]:
        reduced.putpixel((i, j), PIXEL['BLACK'])
      else:
        reduced.putpixel((i, j), PIXEL['WHITE'])
  return reduced

# Replace pixels on array into image
def replace_image(img, array):
  for i in range(len(array)):
    for j in range(len(array[0])):
      # If array index is 1 and pixel in image is not black
      if (array[i][j] == 1) and (img.getpixel((i, j)) != PIXEL['BLACK']):
        img.putpixel((i, j), PIXEL['PATH'])
  return img

# Get data of a point type of pixels into image
def get_point(img, type):
  points = []
  for i in range(img.size[0]):
    for j in range(img.size[1]):
      if type == PIXEL['START']:
        if img.getpixel((i, j)) == PIXEL['START']:
          points.append((i, j))
          return points
      elif type == PIXEL['END']:
        if img.getpixel((i, j)) == PIXEL['END']:
          points.append((i, j))
  return points