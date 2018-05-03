import cv2
import io
import picamera
import FaceRecognition
import numpy

# Create a memory stream so image doesn't need to be saved to a file
stream = io.BytesIO()

# Get the picture in low resolution to speed up processing
with picamera.PiCamera() as camera:
    #camera.resolution = (320, 240)
    camera.resolution = (640, 480)
    camera.capture(stream, format='jpeg')

#Convert picture to numpy array
buff = numpy.fromstring(stream.getvalue(), dtype=numpy.uint8)

# Now create an OpenCV image
img = cv2.imdecode(buff, 1)

# May show more then one face
predicted_img = FaceRecognition.predict(img)

cv2.imshow("Subject", predicted_img)

cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.waitKey(1)
cv2.destroyAllWindows()
