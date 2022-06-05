# Python code for Multiple Color Detection
import numpy as np
import cv2

def simple_detect_bbox(img):
    # simple object detection using color
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_bound = np.array([94, 80, 2])  # red [136, 87, 111], green [25, 52, 72], blue [94, 80, 2]
    upper_bound = np.array([120, 255, 255])  # red [180, 255, 255], green [102, 255, 255], blue [120, 255, 255]
    mask = cv2.inRange(hsv, lower_bound, upper_bound)

    # define kernel size
    kernel = np.ones((7, 7), np.uint8)
    # Remove unnecessary noise from mask
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    # Find contours from the mask
    items = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # print(contours)
    contours = items[0] if len(items) == 2 else items[1]

    x, y, w, h = None, None, None, None
    # loop over the contours
    for c in contours:
        area = cv2.contourArea(c)
        if (area > 300):
            x, y, w, h = cv2.boundingRect(c)
            img = cv2.rectangle(img, (x, y),
                                       (x + w, y + h),
                                       (0, 0, 255), 2)

            cv2.putText(img, "object", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (0, 0, 255))
    return img, x, y, w, h

if __name__ == "__main__":
    # Capturing video through webcam
    webcam = cv2.VideoCapture(0)

    # Start a while loop
    while (1):

        # read webcam
        ret, img = webcam.read()
        if ret:
            img_size = [img.shape[1], img.shape[0]]
            img, x, y, w, h = simple_detect_bbox(img)

            # calculate delta x, y
            delta_x = (x - img_size[0] / 2)
            delta_y = -1 * (y - img_size[1] / 2)

            # show image
            cv2.imshow("image", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            webcam.release()
            cv2.destroyAllWindows()
            break
