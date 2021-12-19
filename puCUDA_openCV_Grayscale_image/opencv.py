import time
import cv2
  
image = cv2.imread('dog-cutout-images.jpg')
start_time = time.time()
grayimage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
  
cv2.imwrite("final_cpu.png",grayimage)
print("--- %s seconds ---" % (time.time() - start_time))

