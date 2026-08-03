import pygame
import serial
import struct
import time
import os

# Windows Xbox Controller
#   Run in cmd
#   Type python3 -m pip install pygame
#   Only works with bluetooth from my testing
#
# Startup
#   if first time select 1 (nano)
#   sudo crontab -e
#   @reboot /home/pi/Desktop/Project/controller_input.py &

def rumble(turnOn=True, joystick=None):
    if turnOn:
        joystick.rumble(1, 1, 200)
        time.sleep(1)
        joystick.stop_rumble()
        joystick.rumble(1, 1, 200)
    else:
        joystick.rumble(1, 1, 500)
        time.sleep(0.5)
        joystick.stop_rumble()
        

# Constants
MIN_JOYSTICK_VALUE = 0.2

# Initializes Pygame
pygame.init()
pygame.joystick.init()

# Get Controllers
print("Joystick count: ", pygame.joystick.get_count())
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]

# Waits for controller connection
while len(joysticks) == 0:
    pygame.joystick.quit()
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
    
rumble(joystick=joysticks[0])

# Initializes Serial
serRB = serial.Serial(
    port='/dev/ttyACM0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS
)

serCAM = serial.Serial(
    port='/dev/ttyACM1',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
)

# Loop
active = True

while active:
    # Updates input state
    pygame.event.pump()

    # Initialize Values
    axisY = 0
    axisX = 0

    # Loops through each controller
    for joystick in joysticks:
        
        # Dead man's switch
        if joystick.get_button(6):
            # Y axis
            y = joystick.get_axis(1)    # Index 1 for Xbox controller left stick (forwards and backwards)
            if abs(y) > MIN_JOYSTICK_VALUE:
                axisY = y
                #print("Y:", axisY)
            
            # X axis
            x = joystick.get_axis(3)    # Index 2 for Xbox controller right stick (left and right)
            if abs(x) > MIN_JOYSTICK_VALUE:
                axisX = x
                #print("\t\tX:", axisX)
                
        # Write to port
        motorDrive = int(axisY * 1023)
        
        motorTurn = -int(struct.unpack('<i', serCAM.read(4))[0] / 45 * 1023)
        if not joystick.get_button(7):
            motorTurn = int(axisX * 1023)
        
        messageMD = struct.pack('>cchc',b'M', b'D', motorDrive, b'\n')
        messageMT = struct.pack('>cchc',b'M', b'T', motorTurn, b'\n')
        
        serRB.write(messageMD + messageMT)
        print(motorDrive, motorTurn)
        
        # Close System
        if joystick.get_button(9):  # Index 7 is the Xbox controller menu button (hold down)
            active = False

    # Wait
    # time.sleep(0.02)
    
# Quit
rumble(turnOn=False, joystick=joysticks[0])
pygame.quit()
    
