from tkinter import*
from configparser import ConfigParser
#from smbus2 import SMBus, i2c_msg
#import time
import sys
#from picamera2 import Picamera2, Preview

root=Tk()
root.title("GripPi")
root.minsize(height=500,width=900)
root["bg"]="#0066CC"

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
    
    def tab2():
        homeLabel.destroy()
        clearWorkspaceLabel.destroy()
        powerButton.destroy()
        calibrateButton.destroy()
        #Top of page 2
        label21=Label(root,text="BINS",font=("Times_New_Roman",25))
        label21.pack(side=TOP)
        label22=Label(root,text="1",font=("Times_New_Roman",25),background="#0066CC")
        label22.place(relx=.15,rely=.1)
        label23=Label(root,text="2",font=("Times_New_Roman",25),background="#0066CC")
        label23.place(relx=.50,rely=.1)
        label24=Label(root,text="3",font=("Times_New_Roman",25),background="#0066CC")
        label24.place(relx=.85,rely=.1)
        #time.sleep(5)
        
        def back():
            label21.destroy()
            label22.destroy()
            label23.destroy()
            label24.destroy()
            backButton.destroy()
            itemButtonLeft.destroy()
            itemButtonCenter.destroy()
            itemButtonRight.destroy()
            tab1()
            
        backButton = Button(root,text="BACK",font=("Times_New_Roman",25),command=back,activebackground="red")
        backButton.pack(side=BOTTOM)
        
        itemButtonLeft = Button(root,image=bin1.image,command=lambda: [updateItemButtonLeft()],activebackground="red")
        itemButtonLeft.pack(side=LEFT)
        
        itemButtonCenter = Button(root,image=bin2.image,command=lambda: [updateItemButtonCenter()],activebackground="red")
        itemButtonCenter.pack(side=LEFT)
        
        itemButtonRight = Button(root,image=bin3.image,command=lambda: [updateItemButtonRight()],activebackground="red")
        itemButtonRight.pack(side=LEFT)
        
        def updateItemButtonLeft():
            if itemButtonLeft:
                
                if bin1.isEmpty :
                    itemButtonLeft.configure(image=photo1)
                    bin1.image = photo1
                    config.set('main', 'bin1_isEmpty', 'False')
                    config.set('image', 'bin1_image', 'bin1.png')
                    bin1.isEmpty = False
                    
                elif not bin1.isEmpty :
                    itemButtonLeft.configure(image=empty_bin_image)
                    bin1.image = empty_bin_image
                    config.set('main', 'bin1_isEmpty', 'True')
                    config.set('image', 'bin1_image', 'EmptyBin.png')
                    bin1.isEmpty = True
                
                with open('config.ini', 'w') as f:
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
                    
                with open('config.ini', 'w') as f:
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
                    
                with open('config.ini', 'w') as f:
                    config.write(f)

    homeLabel=Label(root,text="HOME",font=("Times_New_Roman",25))
    homeLabel.pack(side=TOP)
    
    clearWorkspaceLabel=Label(root,text="Please clear the workspace before calibrating!",font=("Times_New_Roman",25))
    clearWorkspaceLabel.place(relx=.5,rely=.6,anchor=CENTER)
    
    powerButton=Button(root,text="POWER",font=("Times_New_Roman",25),command=close_gui,activebackground="red")
    powerButton.pack(side=BOTTOM)
    
    calibrateButton=Button(root,text="CALIBRATE",font=("Times_New_Roman",25),command=tab2,background="green",activebackground="red")
    calibrateButton.place(relx=.5,rely=.5,anchor= CENTER)

tab1()

root.mainloop()
