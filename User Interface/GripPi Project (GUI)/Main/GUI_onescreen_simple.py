from gpiozero import LED
from time import sleep
from guizero import App, Text, PushButton, Picture
import sys

#Designating LEDs
led1 = LED(3)
led2 = LED(4)
led3 = LED(18)
led4 = LED(17)
led5 = LED(27)
led6 = LED(22)

led4.on()
led5.on()
led6.on()

#Designating commands
def toggle1():
    led1.toggle()
    if led1.is_lit:
       button1.image="store.png"
       Text(app, "RETRIEVED", grid=[0,5])
       led4.off()
    else:
       button1.image="retrieve.png"
       Text(app, "STORED", grid=[0,5],height=10, width=10)
       led4.on()

def toggle2():
    led2.toggle()
    if led2.is_lit:
       button2.image="store.png"
       Text(app, "RETRIEVED", grid=[3,5])
       led5.off()
    else:
       button2.image="retrieve.png"
       Text(app, "STORED", grid=[3,5], height=10, width=10)
       led5.on()

def toggle3():
    led3.toggle()
    if led3.is_lit:
       button3.image="store.png"
       Text(app, "RETRIEVED", grid=[6,5])
       led6.off()
    else:
       button3.image="retrieve.png"
       Text(app, "STORED", grid=[6,5], height=10, width=10)
       led6.on()

def close_gui():
  sys.exit()

#Designating GUI color and layout
app = App(title="GripPi Menu", layout="grid", height=1000, width=1000)
app.bg=(0,102, 204)

#Designating buttons for commands
Text(app, "BINS", grid=[3,0], height=10, width=10)
Text(app, "1", grid=[0,3])
Text(app, "2", grid=[3,3])
Text(app, "3", grid=[6,3])
Text(app, "STORED", grid=[0,5], height=10, width=10)
Text(app, "STORED", grid=[3,5], height=10, width=10)
Text(app, "STORED", grid=[6,5], height=10, width=10)
button1 = PushButton(app, command=toggle1, image="retrieve.png", grid=[0,4])
button2 = PushButton(app, command=toggle2, image="retrieve.png", grid=[3,4])
button3 = PushButton(app, command=toggle3, image="retrieve.png", grid=[6,4])
Text(app, "POWER", grid=[3,9])
button4 = PushButton(app, command=close_gui, image="power.png", grid=[3,10])

app.display()