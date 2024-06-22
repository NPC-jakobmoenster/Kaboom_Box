from machine import Pin, PWM, I2C, SPI, Timer
from utime import sleep
from neopixel import NeoPixel
import random
import sys
import ssd1306
from rotary_irq_rp2 import RotaryIRQ
from writer import Writer

# Font
import arial_50


# Global variables

oledI2cScl      = Pin(5)            # I2C for OLED screen, Clock (SSD1306)
oledI2cSda      = Pin(4)            # I2C for OLED screen, Data (SSD1306)
onBoardLED      = Pin("LED", Pin.OUT) # PICO on board LED for debugging
WIDTH = const(128)
HEIGHT = const(64)

# oled screen
oledI2c = I2C(0, scl=oledI2cScl, sda=oledI2cSda, freq=400_000)
oled = ssd1306.SSD1306_I2C(WIDTH, HEIGHT, oledI2c, addr=0x3c) # 128 x 64 pixel, color: MONO_VLSB, I2C address: 0x3c

def initOled():
    # init the OLED screen with level and time + erase anything from noise during start-up
    oled.fill(0)
    oled.show()

# prepare for use
initOled()

oled.text(f"Let start", 0, 0, 1) # write the new level
oled.show()

# rhs = WIDTH -1
# oled.line(rhs - 20, 0, rhs, 20, 1)
# square_side = 10


wri = Writer(oled, arial_50)
yOffset = 14
xOffset= 10

def count(t):
    # oled.fill_rect(yOffset, 0, HEIGHT-yOffset, WIDTH, 0)
    tWidth = wri.stringlen(t)
    maxWidth = 111
    oled.fill_rect(0, yOffset, WIDTH, HEIGHT-yOffset, 0)
    xOffsetExtra = int((maxWidth - tWidth)/2)
    Writer.set_textpos(oled, 14, xOffset + xOffsetExtra)  # verbose = False to suppress console output
    wri.printstring(t)
    oled.show()

for i in [105,104,103,100, 99, 98, 70, 23, 11, 10, 9,8,7,6,5,4,3,2,1,0]:
    count(f"{i}")
    sleep(0.3)