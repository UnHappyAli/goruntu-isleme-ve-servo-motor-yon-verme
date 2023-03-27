import cv2, numpy as np
from collections import deque
import time
import RPi.GPIO as GPIO




# nesne merkezini depolayacak veri tipi
buffer_size = 16  # deque boyutu
pts = deque(maxlen=buffer_size)

# mavi renk aralığı - HSV
blueLower = (84, 98, 0)
blueUpper = (179, 255, 255)

# capture
cap = cv2.VideoCapture(0)
cap.set(3, 480)  # width
cap.set(4, 320)  # height

while True:
    success, imgOriginal = cap.read()

    if success:
        # detayi azaltip noise azaltma
        blurred = cv2.GaussianBlur(imgOriginal, (11, 11), 0)

        # hsv
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        cv2.imshow("HSV Image", hsv)

        # mavi icin maske
        mask = cv2.inRange(hsv, blueLower, blueUpper)
        cv2.imshow("mask Image", mask)

        # maskenin etrafinda kalan gurultuleri sil
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)
        cv2.imshow("mask+erode-dilate", mask)

        # contour
        (contours, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        center = None

        if len(contours) > 0:
            # en buyuk konturu al
            c = max(contours, key=cv2.contourArea)

            # dikdortgene cevir
            rect = cv2.minAreaRect(c)
            ((x, y), (width, height), rotation) = rect

            s = "x: {}, y: {}, width: {}, height: {}, rotation: {}".format(np.round(x),
                                                                           np.round(y),
                                                                           np.round(width),
                                                                           np.round(height),
                                                                           np.round(rotation))

            print(s)

            box = cv2.boxPoints(rect)
            box = np.uint64(box)

            # moment - goruntunun merkezini bulmamiza yarayan yapi
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]),
                      int(M["m01"] / M["m00"]))

            M2 = cv2.moments(c)
            center2 = (int(M2["m10"] / M2["m00"]),
                      int(M2["m01"] / M2["m00"]))





            # contour'u cizdir
            cv2.drawContours(imgOriginal, [box], 0, (0, 255, 255), 2)

            # merkeze nokta ciz
            cv2.circle(imgOriginal, center, 5, (255, 0, 255), -1)

            # bilgileri ekrana yazdir
            cv2.putText(imgOriginal, s, (20, 20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.4, (0, 0, 0), 1)

        # deque (nokta takip)
        pts.appendleft(center)
        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None: continue
            cv2.line(imgOriginal, pts[i - 1], pts[i], (0, 255, 0), 3)








        cv2.imshow("Orjinal Tespit", imgOriginal)

        # Servo motorun bağlı olduğu GPIO pin numarası
        servo_pin = 18

        # GPIO pinlerini BCM modunda ayarla
        GPIO.setmode(GPIO.BCM)

        # Servo motorun bağlı olduğu pinin çıkış olarak ayarla
        GPIO.setup(servo_pin, GPIO.OUT)

        # PWM (Pulse-Width Modulation) nesnesi oluştur ve frekansı ayarla
        pwm = GPIO.PWM(servo_pin, 50)

        # PWM sinyalini başlat
        pwm.start(0)


        # Servo motoru 0 derece pozisyonda başlat
        def set_servo_angle(angle):
            duty = angle / 18 + 2
            GPIO.output(servo_pin, True)
            pwm.ChangeDutyCycle(duty)
            time.sleep(1)
            GPIO.output(servo_pin, False)
            pwm.ChangeDutyCycle(0)

        if y >=250 :
            set_servo_angle(25)
            time.sleep(3)
            set_servo_angle(165)
            time.sleep(3)
            set_servo_angle(90)
            time.sleep(10)
            set_servo_angle(165)
            time.sleep(3)
            set_servo_angle(90)
            time.sleep(6)
            set_servo_angle(165)
            time.sleep(3)
            set_servo_angle(90)
            time.sleep(10)
            set_servo_angle(165)
            time.sleep(2)
            pwm.stop()
            GPIO.cleanup()




        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            break
