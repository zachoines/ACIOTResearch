import Jetson.GPIO as GPIO
import time

class servo(object):

    """
        Class to control tilt of a servo

        Params:

        y = mx + b -> Duty_Cycle = a / 180 * direction + b, where direction is angle from 1 to 180

    """

    def __init__(self, pwm, slope, offeset):
        P_SERVO = pwm # adapt to your wiring
        fPWM = 50  # Hz (not higher with software PWM)
        a = 10
        b = 2


    def __del__(self):
        setDirection(0)    
        GPIO.cleanup() 


    def setup(self):
        global pwm
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(P_SERVO, GPIO.OUT)
        pwm = GPIO.PWM(P_SERVO, fPWM)
        pwm.start(0)

    def setDirection(self, direction):
        duty = a / 180 * direction + b
        pwm.ChangeDutyCycle(duty)
        time.sleep(1) # allow to settle
