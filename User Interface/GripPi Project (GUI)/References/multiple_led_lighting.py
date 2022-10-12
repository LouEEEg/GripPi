from gpiozero import LED
from time import sleep
from guizero import App, Text, PushButton
import sys

led1 = LED(3)
led2 = LED(4)
led3 = LED(18)

def toggleLED1():
    led1.toggle()
    if led1.is_lit:
       button1.text="LED OFF"
    else :
       button1.text="LED ON"

def toggleLED2():
    led2.toggle()
    if led2.is_lit:
       button2.text="LED OFF"
    else:
       button2.text="LED ON"

def toggleLED3():
    led3.toggle()
    if led3.is_lit:
       button3.text="LED OFF"
    else:
       button3.text="LED ON"

def close_gui():
  sys.exit()
  
def blink_LEDs():
  count = 0
  while count < 5:
    led1.on()
    led2.on()
    led3.on()
    sleep(1)
    led1.off()
    led2.off()
    led3.off()
    sleep(1)
    count+=1

app = App(title="LED Control", layout="grid", height=600, width=800)

Text(app, "Blue", grid=[0,0])
button1 = PushButton(app, command=toggleLED1, text="ON", width=10,height=3, grid=[1,0])
Text(app, "Green", grid=[0,1])
button2 = PushButton(app, command=toggleLED2, text="ON", width=10,height=3, grid=[1,1])
Text(app, "Red", grid=[0,2])
button3 = PushButton(app, command=toggleLED3, text = "ON", width=10,height=3, grid=[1,2])
Text(app, "Blink LEDs", grid=[0,3])
button4 = PushButton(app, command=blink_LEDs, text="Blink LEDs", width=10,height=3, grid=[1,3])
button5 = PushButton(app, command=close_gui, text="Close", grid=[1,4])

app.display()