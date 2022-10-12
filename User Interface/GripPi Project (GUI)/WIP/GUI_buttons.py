from gpiozero import LED
from time import sleep
from guizero import App, Text, Picture, PushButton
import sys

led1 =LED(3)
led2 = LED(4)
led3 = LED(18)

def switch_on():
    print("On")
    led1.on()

#def toggleLED1():
    #led1.toggle()
    #if led1.is_lit:
       #button1.image="toothbrush.png"
    #else :
       #button1.image="toothbrush.png"

#def toggleLED2():
    #led2.toggle()
    #if led2.is_lit:
       #button2.image="medication.png"
    #else:
       #button2.image="medication.png"

#def toggleLED3():
    #led3.toggle()
    #if led3.is_lit:
       #button3.image="phone.png"
    #else:
       #button3.image="phone.png"
    
# def switch_off():
#     print("OFF")
#     led1.off()
#app = App(title="GripPi")

#message = Text(app, text="Robot Manipulator")

#app.bg=(180, 216, 90)

#toothbrush = Picture(app, image="toothbrush.png")

#button = PushButton(app, switch_on, image="toothbrush.png")

app = App(title="GripPi Menu")
app.bg=(180, 216, 90)
button1= PushButton (app, command=switch_on, image="toothbrush.png", width=500, height=200)
button2= PushButton (app, command=switch_on, image="medication.png", width=500, height=200)
button3= PushButton (app, command=switch_on, image="phone.png", width=500, height=200)
app.display()