import cv2
import numpy as np

# Open Pi Camera (mapped to /dev/video0)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ Failed to grab frame")
        break

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red traffic sign range (from RGB 238,39,55)
    lower_red = np.array([170, 150, 100])
    upper_red = np.array([180, 255, 255])

    # Green traffic sign range (from RGB 68,214,44)
    lower_green = np.array([100, 150, 100])
    upper_green = np.array([120, 255, 255])

    # Magenta parking block range (from RGB 255,0,255)
    lower_magenta = np.array([140, 150, 100])
    upper_magenta = np.array([160, 255, 255])

    # Create masks
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_magenta = cv2.inRange(hsv, lower_magenta, upper_magenta)

    # Process colors
    for mask, color_name, box_color in [
        (mask_red, "Red Sign", (0, 0, 255)),
        (mask_green, "Green Sign", (0, 255, 0)),
        (mask_magenta, "Magenta Block", (255, 0, 255))
    ]:
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 500:  # ignore small noise
                x, y, w, h = cv2.boundingRect(cnt)
                cv2.rectangle(frame, (x, y), (x+w, y+h), box_color, 2)
                cv2.putText(frame, color_name, (x, y-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)

    # Show result
    cv2.imshow("Traffic Sign & Parking Detection", frame)

    # Quit on 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
