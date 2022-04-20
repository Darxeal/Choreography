import numpy as np
from PIL import Image
from rlutilities.linear_algebra import vec3, vec2


def convert_img_to_shape(image: str, pixel_size=1):
    """
    Black pixels on image are converted into a list of vec3 which 
    can be used for state-setting for drawing what was in the image.
    """
    shape = []

    img = np.asarray(Image.open(image))
    img = np.transpose(img, (1, 0, 2))

    for y, row in enumerate(img):
        for x, pixel in enumerate(row):
            if np.all(pixel[:3] == np.array([0, 0, 0])):
                shape.append(vec2(y * pixel_size, x * pixel_size))

    return shape
