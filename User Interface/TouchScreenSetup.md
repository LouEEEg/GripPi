These steps will allow the user to add touchscreen capabilities to their Raspberry Pi and UC-430 device. 

Step 1: Download the driver

git clone https://github.com/UCTRONICS/UCTRONICS_LCD35_HDMI_RPI.git

Step 2: Go to the driver path

cd UCTRONICS_LCD35_HDMI_RPI/Raspbian/

Step 3: Get the permisson

sed -i -e 's/\r$//' *.sh
chmod +x *.sh

Step 4: install the driver

Rotation 180

sudo ./install_uc_touch_180.sh

Credit: https://github.com/UCTRONICS/UCTRONICS_LCD35_HDMI_RPI
