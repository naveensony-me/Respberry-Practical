import RPi.GPIO as GPIO
import time
import smbus
import cv2
import numpy as np
import pygame

# Define GPIO pins
BUTTON_1_PIN = 17
BUTTON_2_PIN = 18
BUTTON_3_PIN = 27
BUZZER_PIN = 22

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_1_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_2_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUTTON_3_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(BUZZER_PIN, GPIO.OUT)

# Initialize LCD
bus = smbus.SMBus(1) # For revision 1 Raspberry Pi, change to 0
lcd_addr = 0x3f # I2C address of the LCD
lcd_cols = 16 # Number of columns on the LCD
lcd_rows = 2 # Number of rows on the LCD

# Initialize pygame for buzzer
pygame.mixer.init()

# Load the pre-trained face detection classifier
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')

# Dummy database of known faces (you should replace this with your own database)
known_faces = {
    "John": np.random.rand(128),
    "Alice": np.random.rand(128),
    # Add more known faces as needed
}

# Function to display text on LCD
def lcd_display(text):
    lcd_byte(0x01, 0)
    lcd_string(text)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | 0x08
    bits_low = mode | ((bits << 4) & 0xF0) | 0x08
    bus.write_byte(lcd_addr, bits_high)
    time.sleep(0.0001)
    bus.write_byte(lcd_addr, bits_low)
    time.sleep(0.0001)

def lcd_string(message):
    message = message.ljust(lcd_cols, " ")
    for i in range(lcd_cols):
        lcd_byte(ord(message[i]), 0x01)

# Function to perform face detection and recognition
def authenticate():
    # Capture image from laptop camera
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    cap.release()

    # Convert image to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the image
    faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

    # Iterate through detected faces
    for (x, y, w, h) in faces:
        # Extract face region
        face_roi = gray[y:y+h, x:x+w]

        # Dummy implementation of face recognition
        # Calculate facial features (you should replace this with a real face recognition algorithm)
        current_face_features = np.random.rand(128)

        # Compare with known faces
        for name, features in known_faces.items():
            # Calculate similarity score
            similarity_score = np.dot(features, current_face_features)

            # Threshold for recognition
            if similarity_score > 0.6:  # You may need to adjust this threshold
                print("Authenticated as:", name)
                return True

    print("Face not recognized")
    return False

# Main function
def main():
    lcd_display("Welcome to EVM")

    while True:
        # Check for button presses
        if GPIO.input(BUTTON_1_PIN) == GPIO.LOW:
            # Perform authentication
            if authenticate():
                # Display result on LCD
                lcd_display("Vote for Candidate 1")
                # Increment vote count and trigger buzzer
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(BUZZER_PIN, GPIO.LOW)
        elif GPIO.input(BUTTON_2_PIN) == GPIO.LOW:
            if authenticate():
                lcd_display("Vote for Candidate 2")
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(BUZZER_PIN, GPIO.LOW)
        elif GPIO.input(BUTTON_3_PIN) == GPIO.LOW:
            if authenticate():
                lcd_display("Vote for Candidate 3")
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(BUZZER_PIN, GPIO.LOW)

# Run the main function
if _name_ == "_main_":
    try:
        main()
    except KeyboardInterrupt:
        GPIO.cleanup()
