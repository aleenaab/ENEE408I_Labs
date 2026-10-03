import cv2
from ultralytics import YOLO

def main():
    model = YOLO("yolo11n.pt")
    webcam = cv2.VideoCapture(0)
    while True:
        worked, image = webcam.read()
        if not worked:
            break
        result = model.predict(image, conf=0.4, verbose=False)[0]
        cv2.imshow("YOLO Screen. Press 'x' to exit the screen", result.plot())
        if cv2.waitKey(1) & 0xFF == ord("x"):
            break
    webcam.release()
    cv2.destroyAllWindows()
    
if __name__ == "__main__":
    main()

