import curses
import RPi.GPIO as GPIO
import time





#set GPIO numbering mode
GPIO.setmode(GPIO.BOARD)

#set pin 11 as the claw open/close output, pin 11 50=50Hz pulse
GPIO.setup(11,GPIO.OUT)
GPIO.setup(12,GPIO.OUT)
clawclose = GPIO.PWM(11,50)
clawlift = GPIO.PWM(12,50)

#start PWM w a value of 0 (pulse off)
clawclose.start(0)
clawlift.start(0)
time.sleep(2)

#set duty cycle positions for opened and closed servo
closed = 2
opened = 6
lifted = 3
lowered = 6

#set up curses for keyboard control
screen = curses.initscr()
curses.noecho()
curses.cbreak()
screen.keypad(True)

try:
    while True:
        char = screen.getch()
        if char == ord('q'):
            break
        elif char == curses.KEY_UP:
            print("w")
            
            
        elif char == curses.KEY_DOWN:
            print("s")
            
            
        elif char == curses.KEY_LEFT:
            print("a")
            
            
        elif char == curses.KEY_RIGHT:
            print("d")
            
            
        elif char == curses.KEY_BACKSPACE:
            print("x")
            
            
        elif char == curses.KEY_F1:
            print("opened")
            clawclose.ChangeDutyCycle(opened)
            time.sleep(0.3)
            clawclose.ChangeDutyCycle(0)
            time.sleep(0.7)
        elif char == curses.KEY_F2:
            print("closed")
            clawclose.ChangeDutyCycle(closed)
            time.sleep(0.3)
            clawclose.ChangeDutyCycle(0)
            time.sleep(0.7)
        elif char == curses.KEY_F3:
            print("lowered")
            clawlift.ChangeDutyCycle(lowered)
            time.sleep(0.3)
            clawlift.ChangeDutyCycle(0)
            time.sleep(0.7)
        elif char == curses.KEY_F4:
            print("raised")
            clawlift.ChangeDutyCycle(lifted)
            time.sleep(0.3)
            clawlift.ChangeDutyCycle(0)
            time.sleep(0.7)
finally:
    curses.nocbreak(); screen.keypad(0); curses.echo();
    curses.endwin()
    #clean up servo
    clawclose.ChangeDutyCycle(0)
    clawclose.stop()
    GPIO.cleanup()
    
