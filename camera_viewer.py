import cv2

# Replace with your TurtleBot's IP
robot_ip = "192.168.1.50"

url = f"http://{robot_ip}:8080/stream?topic=/evade/detection_image&type=mjpeg"
cap = cv2.VideoCapture(url)

try:
    while True:
        success, frame = cap.read()
        if not success:
            print("Cannot receive video. Check robot IP and web_video_server.")
            break

        cv2.imshow("TurtleBot Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
finally:
    cap.release()
    cv2.destroyAllWindows()
