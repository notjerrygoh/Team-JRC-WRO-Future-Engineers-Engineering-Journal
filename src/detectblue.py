def detect_blue(frame):
    hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    blue_lower = np.array([100, 150, 50], np.uint8)
    blue_upper = np.array([140, 255, 255], np.uint8)

    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)
    detected = False

    contours, _ = cv2.findContours(blue_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > 500:  # ignore small noise
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(frame, "Blue Detected", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            detected = True

    return detected
