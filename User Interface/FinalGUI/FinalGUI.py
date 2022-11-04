from tkinter import*
from configparser import ConfigParser
#from smbus2 import SMBus, i2c_msg
#import time
import sys
#from picamera2 import Picamera2, Preview


RESOLUTION = "1920x1080"

root=Tk()
root.title("GripPi")
root.geometry(RESOLUTION)
root["bg"]="#0006FC"

config = ConfigParser()
config.read('config.ini')

#Pi camera instance
#picam2 = Picamera2()
#preview_config = picam2.create_preview_configuration(main={"size": (640, 640)})
#picam2.configure(preview_config)
#picam2.start()

photo1 = PhotoImage(file = "bin1.png")
photo2 = PhotoImage(file = "bin2.png")
photo3 = PhotoImage(file = "bin3.png")
background_image = PhotoImage(file = "GripPi.png")
empty_bin_image = PhotoImage(file = "EmptyBin.png")

class gripPiBin:
    def __init__(self, isEmpty, image):
        self.isEmpty = isEmpty
        self.image = image
    

bin1_image = PhotoImage(file = config.get('image', 'bin1_image'))
bin2_image = PhotoImage(file = config.get('image', 'bin2_image'))
bin3_image = PhotoImage(file = config.get('image', 'bin3_image'))

bin1 = gripPiBin(config.getboolean('main', 'bin1_isEmpty'), bin1_image)
bin2 = gripPiBin(config.getboolean('main', 'bin2_isEmpty'), bin2_image)
bin3 = gripPiBin(config.getboolean('main', 'bin3_isEmpty'), bin3_image)


def close_gui():
    sys.exit()

def tab1():
    
    
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- Item Selection Screen - --- --- --- --- --- --- --- 
    # --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- --- ---
    def tab2():
        homeLabel.destroy()
        clearWorkspaceLabel.destroy()
        powerButton.destroy()
        calibrateButton.destroy()
        motionControlButton.destroy()
        #Top of page 2
        label21=Label(root, text="Select an Item", font=("Times_New_Roman",25), anchor="e", justify=CENTER)
        label21.pack(side=TOP)
        
        def back():
            label21.destroy()
            backButton.destroy()
            itemButtonLeft.destroy()
            itemButtonCenter.destroy()
            itemButtonRight.destroy()
            tab1()
            
        backButton = Button(root,text="BACK",font=("Times_New_Roman",25),command=back,activebackground="red")
        backButton.pack(side=BOTTOM)
        
        itemButtonLeft = Button(root,image=bin1.image,command=lambda: [updateItemButtonLeft()],activebackground="red", justify=LEFT)
        itemButtonLeft.pack(side=LEFT)
        
        itemButtonCenter = Button(root,image=bin2.image,command=lambda: [updateItemButtonCenter()],activebackground="red", justify=CENTER)
        itemButtonCenter.pack(side=LEFT)
        
        itemButtonRight = Button(root,image=bin3.image,command=lambda: [updateItemButtonRight()],activebackground="red", justify=RIGHT)
        itemButtonRight.pack(side=LEFT)
        
        def updateItemButtonLeft():
            if itemButtonLeft:
                
                if bin1.isEmpty :
                    itemButtonLeft.configure(image=photo1)
                    bin1.image = photo1
                    bin1.isEmpty = False
                    config.set('main', 'bin1_isEmpty', 'False')
                    config.set('image', 'bin1_image', 'bin1.png')
                    
                elif not bin1.isEmpty :
                    itemButtonLeft.configure(image=empty_bin_image)
                    bin1.image = empty_bin_image
                    bin1.isEmpty = True
                    config.set('main', 'bin1_isEmpty', 'True')
                    config.set('image', 'bin1_image', 'EmptyBin.png')
                    
                
                with open('config.ini', 'w', encoding = "locale") as f:
                    config.write(f)
                
                
        def updateItemButtonCenter():
            if itemButtonCenter:
                
                if bin2.isEmpty :
                    itemButtonCenter.configure(image=photo2)
                    bin2.image = photo2
                    config.set('main', 'bin2_isEmpty', 'False')
                    config.set('image', 'bin2_image', 'bin2.png')
                    bin2.isEmpty = False
                    
                elif not bin2.isEmpty :
                    itemButtonCenter.configure(image=empty_bin_image)
                    bin2.image = empty_bin_image
                    config.set('main', 'bin2_isEmpty', 'True')
                    config.set('image', 'bin2_image', 'EmptyBin.png')
                    bin2.isEmpty = True
                    
                with open('config.ini', 'w', encoding = "locale") as f:
                    config.write(f)
                
                
        def updateItemButtonRight():
            if itemButtonRight:
                
                if bin3.isEmpty :
                    itemButtonRight.configure(image=photo3)
                    bin3.image = photo3
                    config.set('main', 'bin3_isEmpty', 'False')
                    config.set('image', 'bin3_image', 'bin3.png')
                    bin3.isEmpty = False
                    
                elif not bin3.isEmpty :
                    itemButtonRight.configure(image=empty_bin_image)
                    bin3.image = empty_bin_image
                    config.set('main', 'bin3_isEmpty', 'True')
                    config.set('image', 'bin3_image', 'EmptyBin.png')
                    bin3.isEmpty = True
                    
                with open('config.ini', 'w', encoding = "locale") as f:
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
        
        
        background_label = Label(root, image=background_image)
        background_label.place(x=0, y=0, relwidth=1, relheight=1)
        
        def back():
            backButton.destroy()
            scale.destroy()
            button.destroy()
            scale2.destroy()
            button2.destroy()
            background_label.destroy()
            #label.destroy()
            tab1()
        
        backButton = Button(root,text="BACK",font=("Times_New_Roman",25),command=back,activebackground="red")
        backButton.pack(side=BOTTOM)     
                
        # --- --- Joint 1 --- ---
        def sel():
            selection = "Value = " + str(var.get())
            #label.config(text = selection)

        var = DoubleVar()
        scale = Scale( root, variable = var, orient = HORIZONTAL, length=400, resolution = 0.1, to = 260, bg ="blue", fg="white")
        scale.place(relx=0.2,rely=0.9)

        button = Button(root, text="Set Joint 1 Position", command=sel)
        button.place(relx=0.23,rely=0.88, anchor= CENTER)

         
        # --- --- Joint 2 --- ---
        def sel2():
            selection2 = "Value = " + str(var.get())
            #label.config(text = selection)

        var2 = DoubleVar()
        scale2 = Scale( root, variable = var2, orient = HORIZONTAL, length=400, resolution = 0.1, to = 260, bg ="blue", fg="white")
        scale2.place(relx=0.18,rely=0.6)

        button2 = Button(root, text="Set Joint 2 Position", command=sel2)
        button2.place(relx=0.18,rely=0.57)

        #label = Label(root)
        #label.pack()       
        

    
    homeLabel=Label(root,text="GripPi",font=("Times_New_Roman",25), anchor="e", justify=CENTER)
    homeLabel.pack(side=TOP)
    
    clearWorkspaceLabel=Label(root,text="Please clear the workspace before calibrating!",font=("Times_New_Roman",25))
    clearWorkspaceLabel.place(relx=.5,rely=.6,anchor=CENTER)
    
    powerButton=Button(root,text="POWER",font=("Times_New_Roman",25),command=close_gui,activebackground="red")
    powerButton.pack(side=BOTTOM)
    
    calibrateButton=Button(root,text="CALIBRATE",font=("Times_New_Roman",25),command=tab2,background="green",activebackground="red")
    calibrateButton.place(relx=.5,rely=.5,anchor= CENTER)
    
    motionControlButton=Button(root,text="GripPi Motion Control",font=("Times_New_Roman",25),command=screenMotionControl,background="green",activebackground="red")
    motionControlButton.place(relx=0.9,rely=.9,anchor= CENTER)
    

tab1()

root.mainloop()
