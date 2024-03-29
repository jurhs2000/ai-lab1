from PIL import Image

# Matrix size to resize image
RESIZE = (400, 400)
# Threshold percentage of white pixels
THRESHOLD_PERCENTAGE = 0.9
# Threshold percentage of colors pixels
COLOR_THRESHOLD_PERCENTAGE = 0.6

# Pixels types
PIXEL = {
  'BLACK': (0, 0, 0),
  'WHITE': (255, 255, 255),
  'THRESHOLD': (int(255 * THRESHOLD_PERCENTAGE), int(255 * THRESHOLD_PERCENTAGE), int(255 * THRESHOLD_PERCENTAGE)),
  'START': (int(255 * COLOR_THRESHOLD_PERCENTAGE), int(255 - (255 * COLOR_THRESHOLD_PERCENTAGE)), int(255 - (255 * COLOR_THRESHOLD_PERCENTAGE))),
  'END': (int(255 - (255 * COLOR_THRESHOLD_PERCENTAGE)), int(255 * COLOR_THRESHOLD_PERCENTAGE), int(255 - (255 * COLOR_THRESHOLD_PERCENTAGE))),
  'PATH': (0, 0, 255),
  'EXPLORED': (255, 255, 0),
  'FRONTIER': (255, 0, 255)
}

# Read image
def read_image(path):
  return Image.open(path).convert('RGB')

# Write image to file
def write_image(img):
  img.save('output/output.bmp')

# Reduce all pixels of image in 50x50 matrix
def reduce_image(img):
  reduced = img.resize(RESIZE)
  startpoints = []
  endpoints = []
  # convert all pixels of Image to black if they are less than 50% white
  for i in range(reduced.size[0]):
    for j in range(reduced.size[1]):
      pixel = reduced.getpixel((i, j))
      # Start is red, so [0] should be greater and [1] and [2] should be minor
      if pixel[0] > PIXEL['START'][0] and pixel[1] < PIXEL['START'][1] and pixel[2] < PIXEL['START'][2]:
        reduced.putpixel((i, j), PIXEL['START'])
        startpoints.append((i, j))
      # End is green, so [1] should be greater and [0] and [2] should be minor
      elif pixel[0] < PIXEL['END'][0] and pixel[1] > PIXEL['END'][1] and pixel[2] < PIXEL['END'][2]:
        reduced.putpixel((i, j), PIXEL['END'])
        endpoints.append((i, j))
      # If pixel is less than 225
      elif pixel[0] < PIXEL['THRESHOLD'][0] and pixel[1] < PIXEL['THRESHOLD'][1] and pixel[2] < PIXEL['THRESHOLD'][2]:
        reduced.putpixel((i, j), PIXEL['BLACK'])
      else:
        reduced.putpixel((i, j), PIXEL['WHITE'])
  return reduced, startpoints, endpoints

# Replace pixels on array into image
def replace_image(img, path, explored, frontier):
  for node in explored:
    # If pixel in image is not black
    if img.getpixel((node.state[0], node.state[1])) != PIXEL['BLACK'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['END'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['START']:
      img.putpixel((node.state[0], node.state[1]), PIXEL['EXPLORED'])
  for node in frontier:
    # If pixel in image is not black
    if img.getpixel((node.state[0], node.state[1])) != PIXEL['BLACK'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['END'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['START']:
      img.putpixel((node.state[0], node.state[1]), PIXEL['FRONTIER'])
  for node in path:
    # If pixel in image is not black
    if img.getpixel((node.state[0], node.state[1])) != PIXEL['BLACK'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['END'] and img.getpixel((node.state[0], node.state[1])) != PIXEL['START']:
      img.putpixel((node.state[0], node.state[1]), PIXEL['PATH'])
  return img
