import serial
from ultralytics import YOLO 
import cv2 as cv 

arduinoData = serial.Serial("COM8", 9600 , timeout= 1) 

# Detection counter 
detection_counter = 0 
nothing_counter = 0 
DETECTION_THRESHOLD = 10 # Detections needed to send "a" 
NOTHING_THRESHOLD = 40 # No detections needed to send "n" 

def get_value():
    arduinoData.write(b'a') 
    Data = arduinoData.readline().decode('ascii') 
    return Data 

def nothing(): 
    arduinoData.write(b'n') 
    Data = arduinoData.readline().decode('ascii') 
    return Data 

model = YOLO("D:\\Pycharm\\Capstone\\train_data\\best.pt") 

#cap = cv.VideoCapture(0) 
cap = cv.VideoCapture("http://192.168.43.175" + ":81/stream") 
cap.set(3, 1280) # Set width 
cap.set(4, 720) # Set height 

class_names = {0: "Boar", 1: "Monkey"} 

while cap.isOpened(): 
    ret, frame = cap.read() # Read a frame from the webcam 
    if not ret: 
        print("Failed to grab frame. Exiting ... ") 
        break 

    # Run YOLO predictions on the frame 
    # preprocessed_frame = preprocess_frame(frame) 
    results = model(frame, classes=[0, 1], line_width=3) # type: ignore 
    detection_flag = False 
    # Visualize results on the frame 
    for result in results: 
        for box in result.boxes: 
            # Extract bounding box details 
            xl, yl, x2, y2 = map(int, box.xyxy[0]) # Bounding box coordinates conf = box.conf[0] # Confidence score 
            els= int(box.cls[0]) # Class index 
            # Draw the bounding box 
            color= ( (0, 255, 0) if els== 0 else (0, 0, 255) ) # Green for Basketball, Red for Volleyball 
            cv.rectangle(frame, (xl, yl), (x2, y2), color, 2)
            # Add the label 
            label= f"{class_names[cls]} {conf:.2f}" # type: ignore 
            cv.putText(frame, label, (xl, yl - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, color, 2 ) 
            detection_flag = True 

        # Increment or reset counters based on detection 
            if detection_flag: 
                detection_counter += 1 
                nothing_counter = 0 
                # Reset nothing counter 
                if detection_counter >= DETECTION_THRESHOLD: 
                    try:
                        print(get_value()) # Send 'a' to Arduino after detection threshold 
                        detection_counter= 0 # Reset detection counter 
                    except Exception as e: 
                        print(f"Failed to send data to Arduino: {e}")
            else: 
                nothing_counter += 1 
                detection_counter= 0 # Reset detection counter 
                if nothing_counter >= NOTHING_THRESHOLD:
                    try:
                        print(nothing()) # Send 'n' to Arduino after nothing threshold 
                        nothing_counter = 0 # Reset nothing counter 
                    except Exception as e: 
                        print(f"Failed to send data to Arduino: {e}") 
    # Display the frame with bounding boxes 
    cv.imshow("Real-Time Detection", frame)
    # Exit on pressing 'q' 
    if cv.waitKey(l) & 0xFF==27: 
        break

# Release resources         
cap.release() 
cv.destroyAllWindows()            
