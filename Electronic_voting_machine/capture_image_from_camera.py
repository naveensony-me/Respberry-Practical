import cv2

cam_port = 0
cam = cv2.VideoCapture(cam_port)

inp = input('Enter person name')

while True:
   
    ret, frame = cam.read()
    
    if not ret:
        print("Error: Failed to capture frame from the camera.")
        break
    
   
    if frame.shape[0] > 0 and frame.shape[1] > 0:
       
        cv2.imshow(inp, frame)
        
      
        key = cv2.waitKey(1)
       
        if key != -1:
            cv2.imwrite(inp + ".png", frame)
            print("Image taken")
            break
    else:
        print("Error: Invalid frame dimensions.")
        break

