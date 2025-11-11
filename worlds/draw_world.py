import numpy as np 
from PIL import Image

img = np.ones((200, 200, 3)) * 255
img[:1] = np.zeros((1, 200, 3))
img[-1:] = np.zeros((1, 200, 3))
img[:, :1] = np.zeros((200, 1, 3))
img[:, -1:] = np.zeros((200, 1, 3))
img[:5, :5] = np.zeros((5, 5, 3))
img[:5, -5:] = np.zeros((5, 5, 3))
img[-5:, :5] = np.zeros((5, 5, 3))
img[-5:, -5:] = np.zeros((5, 5, 3))

im = Image.fromarray(img.astype(np.uint8))
im.save('python_image.png')