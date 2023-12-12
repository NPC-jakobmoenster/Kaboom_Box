from machine import Pin, PWM, I2C, SPI, Timer
from utime import sleep
from neopixel import NeoPixel
import random
import sys
import ssd1306
from rotary_irq_rp2 import RotaryIRQ

# Global variables

# colors in RGB
RED     = (20,0,0)
GREEN   = (0,10,0)
BLUE    = (0,0,20)
OFF     = (0,0,0)

# dictionaries for out and input that hold the information on [letter byte, letter address, chosen/connected, neopixel adress]
lettersOutDict = {"A":[3, 0x02, False, 0],  "B":[3, 0x04, False, 1],   "C":[3, 0x08, False, 2],   "D":[3, 0x10, False, 3],   "E":[3, 0x20, False, 4],  "F": [3, 0x40, False, 5],  "G": [3, 0x80, False, 6],  "H":[2, 0x01, False, 7],
                  "I":[2, 0x02, False, 8],  "J":[2, 0x04, False, 9],   "K":[2, 0x08, False, 10],  "L":[2, 0x10, False, 11],  "M":[2, 0x20, False, 12], "N": [2, 0x40, False, 13], "O": [2, 0x80, False, 14], "P":[1, 0x01, False, 15],
                  "Q":[1, 0x02, False, 16], "R":[1, 0x04, False, 17],  "S":[1, 0x08, False, 18],  "T":[1, 0x10, False, 19],  "U":[1, 0x20, False, 20], "V": [1, 0x40, False, 21], "W": [1, 0x80, False, 22], "X":[0, 0x01, False, 23],
                  "Y":[0, 0x02, False, 24], "Z":[0, 0x04, False, 25], "AE":[0, 0x08, False, 26], "OE":[0, 0x10, False, 27], "AA":[0, 0x20, False, 28]}

lettersInDict =  {"A":[0, 0x04, False, 29], "B":[0, 0x02, False, 30],  "C":[1, 0x04, False, 31],  "D":[1, 0x02, False, 32],  "E":[2, 0x04, False, 33], "F": [2, 0x02, False, 34], "G": [3, 0x04, False, 35], "H":[3, 0x02, False, 36],
                  "I":[0, 0x08, False, 37], "J":[0, 0x01, False, 38],  "K":[1, 0x08, False, 39],  "L":[1, 0x01, False, 40],  "M":[2, 0x08, False, 41], "N": [2, 0x01, False, 42], "O": [3, 0x08, False, 43], "P":[3, 0x01, False, 44],
                  "Q":[0, 0x10, False, 45], "R":[0, 0x80, False, 46],  "S":[1, 0x10, False, 47],  "T":[1, 0x80, False, 48],  "U":[2, 0x10, False, 49], "V": [2, 0x80, False, 50], "W": [3, 0x10, False, 51], "X":[3, 0x80, False, 52],
                  "Y":[0, 0x20, False, 53], "Z":[0, 0x40, False, 54], "AE":[1, 0x20, False, 55], "OE":[1, 0x40, False, 56], "AA":[2, 0x20, False, 57]}

# tuple to hold alphabet for looping later
letters = ('A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 'AE', 'OE', 'AA')

letterOutBuffer=bytearray(4)    # buffer to hold the output letter information to write on SPI
letterInBuffer=bytearray(4)     # buffer to hold the output letter information to write on SPI

levelMultiplier = 2         # number of letters per level
timeLeftReset = 60          # initial/reset timer
timeLeft = timeLeftReset    # seconds until the "bomb" goes off.
running = False             # Flag for handling if the bomb is started or not.

# PIN setups
lvl1            = Pin(19, Pin.IN)   # input level pin 1
lvl2            = Pin(20, Pin.IN)   # input level pin 2
lvl3            = Pin(21, Pin.IN)   # input level pin 3
startBtn        = Pin(18, Pin.IN)   # Start button
letterOutSck    = Pin(2)            # SPI for letter out, Clock
letterOutMosi   = Pin(3)            # SPI for letter out, MOSI
letterOutMiso   = Pin(0)            # SPI for letter out, MISO
letterOutReset_n        = Pin(7, Pin.OUT, value=0)  # Master Reset (active low) for letter out - start by resetting
letterOutEnableOutput_n = Pin(1, Pin.OUT, value=1)  # Enable output (active low) for letter out - disable outputs on startup
letterInSck     = Pin(10)           # SPI for letter in, Clock
letterInMosi    = Pin(11)           # SPI for letter in, MOSI
letterInMiso    = Pin(8)            # SPI for letter in, MISO
letterInLoadRegisters_n = Pin(9, Pin.OUT, value=1)  # Load input pin (active low) for letter in - do not load on start-up
oledI2cScl      = Pin(5)            # I2C for OLED screen, Clock (SSD1306)
oledI2cSda      = Pin(4)            # I2C for OLED screen, Data (SSD1306)
onBoardLED      = Pin("LED", Pin.OUT) # PICO on board LED for debugging
npPin           = Pin(14, Pin.OUT)  # GPIO14 to output to drive NeoPixels (digital output)
buzzerPin       = Pin(22)           # GPIO22 to output to drive buzzer (digital output)
encoderClk      = 17                # GPIO number for rotary encoder Clock
encoderDt       = 16                # GPIO number for rotary encoder Data




# Setup HW blocks and objects
# lettersOut
letterOut = SPI(0, 400000, sck=letterOutSck, mosi=letterOutMosi, miso=letterOutMiso)    # Create SPI on SPI0
# lettersIn
letterIn = SPI(1, 400000, sck=letterInSck, mosi=letterInMosi, miso=letterInMiso)   # Create SPI on SPI1
# oled screen
oledI2c = I2C(0, scl=oledI2cScl, sda=oledI2cSda, freq=400_000)
oled = ssd1306.SSD1306_I2C(128, 64, oledI2c, addr=0x3c) # 128 x 64 pixel, color: MONO_VLSB, I2C address: 0x3c
# letterLEDs
np = NeoPixel(npPin, len(letters)*2, bpp=3, timing=1)   # create NeoPixel driver for len(letters)*2 pixels, RGB colors
# Buzzer (PWM)
pwm0B = PWM(buzzerPin, freq=2000, duty_u16=0) # 50% duty cycle = duty_u16 = 32768, sound level peak frequency is 4000Hz. Initially turned off. 
# rotary encoder
encoder = RotaryIRQ(pin_num_clk=encoderClk, pin_num_dt=encoderDt, min_val=0, max_val=1000, reverse=False, range_mode=RotaryIRQ.RANGE_BOUNDED) # max time, set to 1000(seconds), can not make them negative
#set start value to timeLeftReset
encoder.set(value=timeLeftReset)



clock = Timer()
def updateTimeLeft(timer):
# Function to count down the timer and show on the display. 
    global timeLeft
    if running:                         # only count down if the bomb is started
        if timeLeft > 0:
            timeLeft = timeLeft -1  
            encoder.set(value=timeLeft) # also update the encoder value, or timeLeft will be overwritten
        updateOledTime()                # update the screen, but only if is running (to save exceution time)
    # print(timeLeft) # debug

# set clock to update time left - period: 1s = 1000ms
clock.init(mode=Timer.PERIODIC, period=1000, callback=updateTimeLeft)

def getLevel():
    # returns the sum of the level pins x the levelMultiplier
    return lvl1.value()*levelMultiplier + lvl2.value()*2*levelMultiplier + lvl3.value()*3*levelMultiplier

def updateOledLevel(level):
    # updates the level on the OLED screen
    oled.fill_rect(0, 0, 128, 10, 0)            # erase the old number
    oled.text(f"KABOOM LEVEL {level}", 0, 0, 1) # write the new level
    oled.show()

def updateOledTime():
    global timeLeft
    # updates the timeLeft on the OLED screen
    oled.fill_rect(0, 20, 128, 10, 0)           # erase the old number
    oled.text(f"Tid: {timeLeft}", 0, 20, 1)     # write the new time
    oled.show()

def initOled():
    # init the OLED screen with level and time + erase anything from noise during start-up
    oled.fill(0)
    oled.show()
    updateOledLevel(getLevel())
    updateOledTime()

def initLetterLEDs():
    # init the Neopixels
    np.fill(OFF)    # fill with OFF pixels
    np.write()      # write data to all pixels

def initBuzzer(): # TODO: consider moving to other core
    # 2 beeps for notify user that system is ready
    pwm0B.duty_u16(32768)    # ON
    sleep(0.1)
    pwm0B.duty_u16(0)        # OFF
    sleep(0.1)
    pwm0B.duty_u16(32768)    # ON
    sleep(0.1)
    pwm0B.duty_u16(0)        # OFF
    
# prepare for use
initOled()
initLetterLEDs()
initBuzzer()

def beep(beepTime=0.5):
    # single beep from the buzzer
    pwm0B.duty_u16(32768)   # ON
    sleep(beepTime)
    pwm0B.duty_u16(0)       # OFF

def ledBomb(defused):
    # light and color show for when the "bomb" goes off, or is defused
    if defused:
        color = GREEN
    else:
        color = RED
    # TODO: turn on rumble
    for i in range(0,10):
        np.fill(color)
        np.write()
        beep(0.5)
        np.fill(OFF)
        np.write()
        sleep(0.5)
        


def setLetterOut(level):
    # Randomly select the letters that needs to be connected to defuse the "bomb"
    # Set the LEDs on the "output" for the user to see.
    
    # Ensure that there are no old chosen letters 
    for letter in letters:
        lettersOutDict[letter][2] = False
    
    # randomly choose the letters based on the level we have. 
    chosenLetters = ["A"]           # DEBUG
    lettersOutDict["A"][2] = True   # DEBUG
    # chosenLetters = []            # keep track of the chosen letters
    while len(chosenLetters)<level: # continue to we have selected level number of letters (that are different)
        letter = random.choice(letters)         # random letter
        if letter in chosenLetters:             # check if we have used it before - if we have do not reuse it.
            pass
        else:
            chosenLetters.append(letter)        # if we have not used it before, add it, and set the selected falg to true.
            lettersOutDict[letter][2] = True 
    
    # Setting the output LEDs
    tempo = 0.2                     # how fast it is scanned
    np.fill(OFF)                    # Ensure all LEDs are off .
    np.write()    
    # go through all letters, light it up, and keep it lit if selected, otherwise turn it off again.
    for letter in letters:
        tempLED = lettersOutDict[letter][3] # get the neopixel address for the letter
        # print(letter, tempLED)
        np[tempLED] = RED                   # make it red
        np.write()                          # and show it
        if lettersOutDict[letter][2]:       # make a beep if the letter is selected
            beep(tempo)
        else:                               # if not selected, turn it off again before moving on
            np[tempLED] = OFF
            sleep(tempo)
            np.write()

def start():
    # when start button is pressed, prepare the "bomb"
    level = getLevel()      # get the selected level (can't be updated anymore)
    updateOledLevel(level)  # Ensure the level is correctly displayed.
    setLetterOut(level)     # Find the letters based on the level, and set them up, incl. the neopixel
    beep(0.4)               # do some beeping as a count down to start.
    sleep(0.2)
    beep(0.4)
    sleep(0.2)
    beep(0.4)
    sleep(0.2)
    return True
##################################################################################################################################
def reset():
    global timeLeft
    timeLeft = timeLeftReset
    encoder.set(value=timeLeftReset)
    updateOledTime()

def scanLetters(level):
    # print("started scanning letters")
    letterOutReset_n.off()                                               # Reset
    sleep(0.01)
    letterOutReset_n.on()                                               # Release the reset
    
    # ensure that there is no letters in the buffer
    for i in range(0, len(letterOutBuffer)):
        letterOutBuffer[i] = 0x00
    # go through each letter
    
    connected = 0
    for letter in letters: #letters
        change = False
        letterOutBuffer[lettersOutDict[letter][0]]=lettersOutDict[letter][1]# set the letter in the output buffer
        letterOut.write(letterOutBuffer)                                    # write to the registers
        letterOutEnableOutput_n.off()                                       # enable outputs on lettersOut
        letterOutBuffer[lettersOutDict[letter][0]]=0x00                     # remove the letter again
        # print(letter)
        # remove the conenction (is case it is removed) - it will be added shortly if it is there. 
        if lettersInDict[letter][2]:
            lettersInDict[letter][2] = False
            change = True
        #Check if letter is correctly connected# #Setup lettersIn
        sleep(0.001)
        # load the registers
        letterInLoadRegisters_n.value(0)
        sleep(0.001)
        letterInLoadRegisters_n.value(1)
        sleep(0.001)
        letterIn.readinto(letterInBuffer)                                   # write to the registers
        # print(letterInBuffer)                                               # debug
        # check if the hex stored in the dict is the same as the read one
        match = letterInBuffer[lettersInDict[letter][0]] == lettersInDict[letter][1]
        if match and lettersOutDict[letter][2]:
            lettersInDict[letter][2] = True
            change = True
            connected = connected + 1
            # print("There is a match")
        # update the LEDs to match if they are connected or not. 
        if change:
            if lettersInDict[letter][2]:
                np[lettersInDict[letter][3]]=GREEN
            else:
                np[lettersInDict[letter][3]]=OFF
            np.write()
    # check if they are all connected - return true if they are
        print(connected,level,letter)
    if connected == level:
        return True
    else:
        return False

level = getLevel()
finished = False
# Main loop
while True:
    if not running:                     # State: Setup
        timeLeftOld = timeLeft 
        timeLeft = encoder.value()
        if timeLeftOld == timeLeft:
            pass
        else:
            updateOledTime()
        oldLevel = level
        level = getLevel()
        if oldLevel == level:
            pass
        else:
            updateOledLevel(level)
        if startBtn.value():
            running = start()
    if running:                         # State: "Bomb"
        finished = scanLetters(level)
        if timeLeft == 0:
            pass
            running = False
            # Times up!
            print("Bomb exploded")
            ledBomb(False)
            reset()
        if finished:
            pass
            running = False
            # Bomb is defused!
            print("Bomb defused")
            ledBomb(True)
            reset()
        # print(running)
