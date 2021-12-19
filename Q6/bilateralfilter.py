# -*- coding: utf-8 -*-
"""
Created on Mon Oct 10 20:14:10 2021

@author: shubh
"""

import cv2
import matplotlib.pyplot as plt

# Read the image.
img = cv2.imread('test.jpg')
# Apply bilateral filter
bilateral = cv2.bilateralFilter(img,10,500,500)

# Save the output.
cv2.imwrite('bilateral2.jpg', bilateral)

#Show both images
image1 = cv2.imread('test.jpg')
image2 = cv2.imread('bilateral2.jpg')
plt.subplot(1, 2, 1)
plt.axis('off')
plt.title("Before")
plt.imshow(image1)
plt.subplot(1, 2, 2)
plt.axis('off')
plt.title("After using bilateral filter")
plt.imshow(image2)
plt.show()