import time
import serial
import cv2

from helpers import *

# Modifiable params
PORT = 'COM4'  # Replace with your Arduino's serial port
BAUD = 921600          # Change to 115200 for Due
cascade=cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
video=cv2.VideoCapture(0)
# Main
if __name__ == '__main__':
    # Open connection to Arduino with a timeout of two seconds
    port = serial.Serial(PORT, BAUD, timeout=2)

    # Report acknowledgment from camera
    getack(port)

    # Wait a spell
    time.sleep(0.2)

    # Send start flag
    sendbyte(port, 1)

    # We'll report frames-per-second
    start = time.time()
    count = 0

    # Loop over images until user hits ESC
    done = False
    while not done:
        check,frame=video.read()
        # Open a temporary file that we'll write to and read from
        tmpfile = open("tmp.jpg", "wb")

        # Loop over bytes from Arduino for a single image
        written = False
        prevbyte = None
        while not done:
            # Read a byte from Arduino
            currbyte = port.read(1)

            # If we've already read one byte, we can check pairs of bytes
            if prevbyte:
                # Start-of-image sentinel bytes: write previous byte to temp file
                if ord(currbyte) == 0xd8 and ord(prevbyte) == 0xff:
                    tmpfile.write(prevbyte)
                    written = True

                # Inside image, write current byte to file
                if written:
                    tmpfile.write(currbyte)

                # End-of-image sentinel bytes: close temp file and display its contents
                if ord(currbyte) == 0xd9 and ord(prevbyte) == 0xff:
                    tmpfile.close()
                    img = cv2.imread("tmp.jpg")
                    if img is not None:
                        # Display the image
                        cv2.imshow("ArduCAM [ESC to quit]", img)
                        if cv2.waitKey(1) == 27:
                            done = True
                            break
                        count += 1
                        break

            # Track previous byte
            prevbyte = currbyte
            gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
            face=cascade.detectMultiScale(gray,scaleFactor=1.1,minNeighbors=6)
            print(type(face))
            for x,y,w,h in fec:
                frame=cv2.rectangle(frame,(x,y),(x+w,y+h),(0,255,255),3)
            cv2.imshow("Video",frame)
            key=cv2.waitKey(1)
            if(key==ord('q')):
                break
    video.release()
    # Send stop flag
    sendbyte(port, 0)

    # Report FPS
    elapsed = time.time() - start
    print(f'{count} frames in {elapsed:.2f} seconds = {count/elapsed:.2f} frames per second')

    # Close the window
    cv2.destroyAllWindows()
