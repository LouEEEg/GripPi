from gpiozero import LED
from time import sleep
from guizero import App, Text, PushButton, Picture, MenuBar
import sys

led1 = LED(3)
led2 = LED(4)
led3 = LED(18)
led4 = LED(17)
led5 = LED(27)
led6 = LED(22)

def file_function():
    print("File option")
    
def edit_function():
    print("Edit option")

def toggleLED1():
    led1.toggle()
    if led1.is_lit:
       button1.image="retrieve.png"
    else :
       button1.image="retrieve.png"

def toggleLED2():
    led2.toggle()
    if led2.is_lit:
       button2.image="store.png"
    else:
       button2.image="store.png"

def close_gui():
  sys.exit()
  
app = App(title="GripPi Menu", layout="grid", height=1000, width=1000)
app.bg=(0,102, 204)

menubar = MenuBar(app,
                  toplevel=["Store", "Retrieve"],
                  options=[
                      [["Storage 1", file_function], ["Storage 2", file_function], ["Storage 3", file_function] ],
                      [["Storage 1", edit_function], ["Storage 2", edit_function], ["Storage 3", edit_function] ]
                    ])
app.display()