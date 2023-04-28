from tkinter import*
from tkinter import messagebox
from configparser import ConfigParser
#from smbus2 import SMBus, i2c_msg
import serial
import time
import sys
from picamera2 import Picamera2, Preview
import cv2
import os

#RESOLUTION = "1920x1080"
RESOLUTION = "1024x768"

root=Tk()
root.title("GripPi")
root.geometry(RESOLUTION)
root["bg"]="#0066FC"
root.attributes('-fullscreen', True)

config = ConfigParser()
config.read('config.ini')

#Pi camera instance
picam2 = Picamera2()
#preview_config = picam2.create_preview_configuration(main={"size": (640, 640)})
preview_config = picam2.create_preview_configuration(main={"size": (341, 341)})
picam2.configure(preview_config)
picam2.start()

photo1 = PhotoImage(file = "bin1.png")
photo2 = PhotoImage(file = "bin2.png")
photo3 = PhotoImage(file = "bin3.png")
#background_image = PhotoImage(file = "GripPi.png")
#background_image = PhotoImage(file = "GripPi_small.png")
#empty_bin_image = PhotoImage(file = "EmptyBin.png")
empty_bin_image = PhotoImage(file = "EmptyBin_small.png")

class gripPiBin:
    def __init__(self, isEmpty, image, states_pick):
        self.isEmpty = isEmpty
        self.image = image
        self.states_pick = states_pick

#START - NEW ADDITIONS - 02/15/2023
class widgets:
    def __init__(self, Button):
        self.Button = Button
          
#END - NEW ADDITIONS - 02/15/2023

def GripPiSerial(GripPiTx):
    GripPiRx=77
    
    if __name__ == '__main__':
        ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
        ser.reset_input_buffer()
        ser.write(str(GripPiTx).encode('utf-8'))
        while GripPiRx == 77:
            while ser.in_waiting > 0:
                GripPiRx = ser.read_until().decode('utf-8').rstrip()
    return GripPiRx

bin1_image = PhotoImage(file = config.get('image', 'bin1_image'))
bin2_image = PhotoImage(file = config.get('image', 'bin2_image'))
bin3_image = PhotoImage(file = config.get('image', 'bin3_image'))

bin1_states_pick = [10,11,3,12,10,5,6,2]
bin2_states_pick = [20,21,3,22,20,5,6,2]
bin3_states_pick = [30,31,3,32,30,5,6,2]

# <x,y,z> where x,y,z are int

bin1_states_place = [8,7,10,12,11,4,12,10,0]
bin2_states_place = [8,7,20,22,21,4,22,20,0]
bin3_states_place = [8,7,30,32,31,4,32,30,0]

gripPi_states_off = [6,97]
gripPi_states_on = [96]

bin1 = gripPiBin(config.getboolean('main', 'bin1_isEmpty'), bin1_image, bin1_states_pick)
bin2 = gripPiBin(config.getboolean('main', 'bin2_isEmpty'), bin2_image, bin2_states_pick)
bin3 = gripPiBin(config.getboolean('main', 'bin3_isEmpty'), bin3_image, bin3_states_pick)

gripPi_calibration = False

#02/27/2023
global Bin_in
Bin_in = True
#02/27/2023

#def close_gui():
    #for x in range(2):
        #homeset = GripPiSerial(gripPi_states_off[x])
    ##time.sleep()
    #sys.exit()
    
def calibrate():
    messagebox.showwarning("WARNING", "Clear workspace before Calibrating")
    homeSet = GripPiSerial(96)
    global gripPi_calibration
    gripPi_calibration = True 

#03/01/2023
def close_gui():
    if(not(gripPi_calibration)):
        homeSet = GripPiSerial(95)
        sys.exit()
    else:  
        for x in range(2):
            homeset = GripPiSerial(gripPi_states_off[x])
        #time.sleep()
        sys.exit()
#03/01/2023

def forget(widget):
    widget.pack_forget

def retrieve(widget):
    widget.pack(side=LEFT)
    
def screenHome():
    
    def isCalibrated():
        if(not(gripPi_calibration)):
            messagebox.showerror("ERROR", "GripPi must be calibrated prior to use!")
        else:
            screenItemSelect()
    
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- Item Selection Screen - --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
    def screenItemSelect():
        
        homeLabel.destroy()
        clearWorkspaceLabel.destroy()
        powerButton.destroy()
        calibrateButton.destroy()
        motionControlButton.destroy()
        binsButton.destroy()

        # --- --- --- Top of page 2 --- --- --- #
        selectItemLabel=Label(root, text="Select an Item", font=("Times_New_Roman",25), anchor="e", justify=CENTER)
        selectItemLabel.pack(side=TOP)
        
                
        #homeSet = GripPiSerial(99)
        
        def back():
            selectItemLabel.destroy()
            backButton.destroy()
            itemButtonLeft.destroy()
            itemButtonCenter.destroy()
            itemButtonRight.destroy()
            screenHome()  
        
        #02/27/2023
        def isReturned():
             global Bin_in
             if(not(Bin_in)):
                 messagebox.showerror("ERROR", "Bin must be returned to storage!")
             else:
                 Bin_in = True
                 back()
        #02/27/2023
                 
        backButton = Button(root,text="BACK",font=("Times_New_Roman",25),command=lambda: [isReturned()],activebackground="red")
        backButton.pack(side=BOTTOM)
        
        itemButtonLeft = Button(root,image=bin1.image,command=lambda: [updateItemButtonLeft()],activebackground="red", justify=LEFT)
        itemButtonLeft.pack(side=LEFT)
        
        itemButtonCenter = Button(root,image=bin2.image,command=lambda: [updateItemButtonCenter()],activebackground="red", justify=CENTER)
        itemButtonCenter.pack(side=LEFT)
        
        itemButtonRight = Button(root,image=bin3.image,command=lambda: [updateItemButtonRight()],activebackground="red", justify=RIGHT)
        itemButtonRight.pack(side=LEFT)
        
        def updateItemButtonLeft():  
            if itemButtonLeft:
                itemButtonCenter.pack_forget()
                itemButtonRight.pack_forget()
                if bin1.isEmpty :
                    homeset = GripPiSerial(1)
                    time.sleep(9)
                    picam2.capture_file("bin1.png")
                    photo1 = PhotoImage(file = "bin1.png")
                    itemButtonLeft.configure(image=photo1)
                    bin1.image = photo1
                    config.set('main', 'bin1_isEmpty', 'False')
                    config.set('image', 'bin1_image', 'bin1.png')
                    #02/27/2023
                    global Bin_in
                    Bin_in = True
                    #02/27/2023
                    bin1.isEmpty = False
                    retrieve(itemButtonCenter)
                    retrieve(itemButtonRight)
                    for x in range(9):
                        homeset = GripPiSerial(bin1_states_place[x])
                    itemButtonLeft.configure(image = photo1)
                      
                elif not bin1.isEmpty :
                    for x in range(8):
                        homeset = GripPiSerial(bin1_states_pick[x])
                    itemButtonLeft.configure(image=empty_bin_image)
                    bin1.image = empty_bin_image
                    config.set('main', 'bin1_isEmpty', 'True')
                    #config.set('image', 'bin1_image', 'EmptyBin.png')
                    config.set('image', 'bin1_image', 'EmptyBin_small.png')
                    #02/27/2023
                    Bin_in = False
                    #02/27/2023
                    bin1.isEmpty = True
                
                with open('config.ini', 'w', encoding = "UTF-8") as f:
                    config.write(f)                                          
                
        def updateItemButtonCenter():
            if itemButtonCenter:
                itemButtonLeft.pack_forget()
                itemButtonRight.pack_forget()
                if bin2.isEmpty :
                    homeset = GripPiSerial(1)
                    time.sleep(9)
                    picam2.capture_file("bin2.png")
                    photo2 = PhotoImage(file = "bin2.png")
                    itemButtonCenter.configure(image=photo2)
                    bin2.image = photo2
                    config.set('main', 'bin2_isEmpty', 'False')
                    config.set('image', 'bin2_image', 'bin2.png')
                    #02/27/2023
                    global Bin_in
                    Bin_in = True
                    #02/27/2023
                    bin2.isEmpty = False
                    retrieve(itemButtonLeft)
                    retrieve(itemButtonRight)
                    for x in range(9):
                        homeset = GripPiSerial(bin2_states_place[x])
                    itemButtonCenter.configure(image = photo2) 
                    
                elif not bin2.isEmpty :
                    for x in range(8):
                        homeset = GripPiSerial(bin2_states_pick[x])
                    itemButtonCenter.configure(image=empty_bin_image)
                    bin2.image = empty_bin_image
                    config.set('main', 'bin2_isEmpty', 'True')
                    #config.set('image', 'bin2_image', 'EmptyBin.png')
                    config.set('image', 'bin2_image', 'EmptyBin_small.png')
                    #02/27/2023
                    Bin_in = False
                    #02/27/2023
                    bin2.isEmpty = True
                    
                with open('config.ini', 'w', encoding = "UTF-8") as f:
                    config.write(f)
                
        def updateItemButtonRight():
            if itemButtonRight:
                itemButtonLeft.pack_forget()
                itemButtonCenter.pack_forget()
                if bin3.isEmpty :
                    homeset = GripPiSerial(1)
                    time.sleep(9)
                    picam2.capture_file("bin3.png")
                    photo3 = PhotoImage(file = "bin3.png")
                    itemButtonRight.configure(image=photo3)
                    bin3.image = photo3
                    config.set('main', 'bin3_isEmpty', 'False')
                    config.set('image', 'bin3_image', 'bin3.png')
                    #02/27/2023
                    global Bin_in
                    Bin_in = True
                    #02/27/2023
                    bin3.isEmpty = False
                    retrieve(itemButtonLeft)
                    retrieve(itemButtonCenter)
                    for x in range(9):
                        homeset = GripPiSerial(bin3_states_place[x])
                    itemButtonRight.configure(image = photo3)
                    
                elif not bin3.isEmpty :
                    for x in range(8):
                        homeset = GripPiSerial(bin3_states_pick[x])
                    itemButtonRight.configure(image=empty_bin_image)
                    bin3.image = empty_bin_image
                    config.set('main', 'bin3_isEmpty', 'True')
                    #config.set('image', 'bin3_image', 'EmptyBin.png')
                    config.set('image', 'bin3_image', 'EmptyBin_small.png')
                    #02/27/2023
                    Bin_in = False
                    #02/27/2023
                    bin3.isEmpty = True
                    
                with open('config.ini', 'w', encoding = "UTF-8") as f:
                    config.write(f)
                
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- Motion Control Screen - --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- 

    def screenMotionControl():
        homeLabel.destroy()
        clearWorkspaceLabel.destroy()
        powerButton.destroy()
        calibrateButton.destroy()
        motionControlButton.destroy()
        binsButton.destroy()     
        
        #background_label = Label(root, image=background_image)
        #background_label.place(x=0, y=0, relwidth=1, relheight=1)
        
        def back():
            backButton.destroy()
            MeasureButtonLeft.destroy()
            MeasureButtonCenter.destroy()
            MeasureButtonRight.destroy()
            screenHome()
            
        def MeasureLeft():
            cmd1 = "python '/home/grippi/GripPi Project (GUI)/Final/OpenCV_test.py' --image '/home/grippi/GripPi Project (GUI)/Final/bin1.png' --width 0.7900"
            os.system(cmd1)
        
        def MeasureCenter():
            cmd2 = "python '/home/grippi/GripPi Project (GUI)/Final/OpenCV_test.py' --image '/home/grippi/GripPi Project (GUI)/Final/bin2.png' --width 0.7900"
            os.system(cmd2)
       
        def MeasureRight():
            cmd3 = "python '/home/grippi/GripPi Project (GUI)/Final/OpenCV_test.py' --image '/home/grippi/GripPi Project (GUI)/Final/bin3.png' --width 0.7900"
            os.system(cmd3)
       
        backButton = Button(root,text="BACK",font=("Times_New_Roman",25),command=lambda: [back()],activebackground="red")
        backButton.pack(side=BOTTOM)
        
        MeasureButtonLeft = Button(root,image=bin1.image,command=lambda: [MeasureLeft()],activebackground="red", justify=LEFT)
        MeasureButtonLeft.pack(side=LEFT)
       
        MeasureButtonCenter = Button(root,image=bin2.image,command=lambda: [MeasureCenter()],activebackground="red", justify=CENTER)
        MeasureButtonCenter.pack(side=LEFT)
        
        MeasureButtonRight = Button(root,image=bin3.image,command=lambda: [MeasureRight()],activebackground="red", justify=RIGHT)
        MeasureButtonRight.pack(side=LEFT)

        # --- --- Input Box --- ---
        #joint1 = Label(root, text = "Joint 1 Position")
        #joint1.place(relx=0.75, rely=0.5 + 0.05)
        #joint2 = Label(root, text = "Joint 2 Position")
        #joint2.place(relx=0.75, rely=0.5 + 0.10)
        #joint3 = Label(root, text = "Joint 3 Position")
        #joint3.place(relx=0.75, rely=0.5 + 0.15)
        #joint4 = Label(root, text = "Joint 4 Position")
        #joint4.place(relx=0.75, rely=0.5 + 0.20)
        #joint5 = Label(root, text = "Joint 5 Position")
        #joint5.place(relx=0.75, rely=0.5 + 0.25)
        
        #e_1 = Entry(root)
        #e_1.place(relx=0.825, rely=0.5 + 0.05)
        #e_2 = Entry(root)
        #e_2.place(relx=0.825, rely=0.5 + 0.10)
        #e_3 = Entry(root)
        #e_3.place(relx=0.825, rely=0.5 + 0.15)
        #e_4 = Entry(root)
        #e_4.place(relx=0.825, rely=0.5 + 0.20)
        #e_5 = Entry(root)
        #e_5.place(relx=0.825, rely=0.5 + 0.25)
        
        #joint1data = Button(root, text = "Enter", command = lambda: [jointSend(e_1)], activebackground="green")
        #joint1data.place(relx=0.925, rely = 0.5 + 0.05)
        #joint2data = Button(root, text = "Enter", command = lambda: [jointSend(e_2)], activebackground="green")
        #joint2data.place(relx=0.925, rely = 0.5 + 0.10)
        #joint3data = Button(root, text = "Enter", command = lambda: [jointSend(e_3)], activebackground="green")
        #joint3data.place(relx=0.925, rely = 0.5 + 0.15)
        #joint4data = Button(root, text = "Enter", command = lambda: [jointSend(e_4)], activebackground="green")
        #joint4data.place(relx=0.925, rely = 0.5 + 0.20)
        #joint5data = Button(root, text = "Enter", command = lambda: [jointSend(e_5)], activebackground="green")
        #joint5data.place(relx=0.925, rely = 0.5 + 0.25)
    
    homeLabel=Label(root,text="GripPi",font=("Times_New_Roman",25), anchor="e", justify=CENTER)
    homeLabel.pack(side=TOP)
    
    clearWorkspaceLabel=Label(root,text="Please clear the workspace before calibrating!",font=("Times_New_Roman",25))
    clearWorkspaceLabel.place(relx=.5,rely=.5,anchor=CENTER)
    
    powerButton=Button(root,text="POWER",font=("Times_New_Roman",25),command=close_gui,activebackground="red")
    powerButton.pack(side=BOTTOM)
    
    calibrateButton=Button(root,text="CALIBRATE",font=("Times_New_Roman",75),command = lambda: [calibrate()],background="green",activebackground="red")
    calibrateButton.place(relx=.5,rely=.25,anchor= CENTER)
    
    motionControlButton=Button(root,text="Object Measurements",font=("Times_New_Roman",25),command=screenMotionControl,background="green",activebackground="red")
    motionControlButton.place(relx=0.8,rely=.95,anchor= CENTER)
    
    binsButton=Button(root, text="BINS", font=("Times_New_Roman",100), command = lambda: [isCalibrated()], background="green",activebackground="red")
    binsButton.place(relx=.5,rely=0.725,anchor= CENTER)
    

screenHome()

root.mainloop()
