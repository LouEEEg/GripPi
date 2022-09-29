from gpiozero import LED
from time import sleep
from guizero import App, Text, PushButton
import sys

myled =LED(4)
  
def switch_on():
    print("On")
    myled.on()
    
def switch_off():
    print("OFF")
    myled.off()
    
def close_gui():
    sys.exit()
    myled.off()
    
def blink_LED():
    count=0
    while count<5:
        myled.on()
        sleep(1)
        myled.off()
        sleep(1)
        count +=1
        
app = App(title="LED Control")
button1= PushButton (app, command=switch_on, text="LED ON", width=10, height=3)
button2= PushButton (app, command=switch_off, text="LED OFF", width=10, height=3)
button3= PushButton (app, command=blink_LED, text="Blink LED", width=10, height=3)
button4= PushButton (app, command=close_gui, text="Close", width=10, height=3)

app.display()
