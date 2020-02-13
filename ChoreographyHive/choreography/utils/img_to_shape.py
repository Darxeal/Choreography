import numpy as np
from PIL import Image
from rlutilities.linear_algebra import vec3


def convert_img_to_shape(image: str, pixel_size=150):
    """
    Black pixels on image are converted into a list of vec3 which 
    can be used for state-setting for drawing what was in the image.
    """
    shape = []

    img = np.asarray(Image.open(image))

    for y, row in enumerate(img):
        for x, pixel in enumerate(row):
            if np.all(pixel == np.array([0, 0, 0])):
                shape.append(vec3(x * pixel_size, y * pixel_size, 18))

    return shape
