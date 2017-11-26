import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)

#Pin setup for ultrasonic sensor
TRIG = 23 
ECHO = 24
GPIO.setup(TRIG,GPIO.OUT)
GPIO.setup(ECHO,GPIO.IN)
GPIO.setup(27, GPIO.OUT)

p = GPIO.PWM(27,100)#(motor pin,frequency)
p.start(0)


GPIO.output(TRIG,False)


time.sleep(2)
    
try:
    while True: #begin ultrasonic sensor read
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
            pulse_start = time.time()

        while GPIO.input(ECHO)==1:
            pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150

        distance = round(distance, 2) #distance in centimeters
        if distance <= 40: #if object infront of car is 40 cm or less away then go to slow stop
            print("STOP")#tell other car to decelerate
            for i in range(100):#decelerate
                p.ChangeDutyCycle(100 - i)
                time.sleep(0.02)
            break
        else:
            p.ChangeDutyCycle(100)#continue high speed
            
        print("Distance:",distance,"cm")#output distance

        time.sleep(.1)
    print("done")#car stopped
        
except KeyboardInterrupt:
    p.stop()
    GPIO.cleanup()
