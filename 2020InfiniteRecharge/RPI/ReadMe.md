# RasPI 
## Downloads

Download BalenaEtcher
[BalenaEtcher Download Link](https://www.balena.io/etcher/)

Download Raspian Buster Lite ZIP
[Raspian BusterLite Download](https://www.raspberrypi.org/downloads/raspbian/)

## Flash SD
Extract Raspbian Image

Using Etcher select the Raspbian image, flash to the SD card

## Installing Opencv
First install pip3

Run the following command: sudo apt install python3-pip

Then install opencv with his command: pip3 install opencv-python==3.4.6.27

Note: There is a 4.1 version of OpenCV but I have not tried installing that version as the version above as been the one to work for me

Here is a full list of commands I did since start up
    
    sudo apt-get update
    
    sudo apt install python3-pip
    
    pip3 install opencv-contrib-python==4.1.0.25
    
    sudo apt install libjasper1
    
    sudo apt install libjasper-dev
    
    sudo apt install libqtgui4
   
    sudo apt install libqt4-test
    
    sudo apt install libatlas3-base
   
    sudo apt-get install libhdf5-dev


## Fixing ImportError: no module named cv2
In geany go to Build->Set Build Commands

In the Set Build Commands window, under python commands the label in compile line will be python m py_complie change it to python3 m py_compile

Under execute command also change the line python "%f" into python3 "%f"

## Making script run on start up
**!!!!MAKE SURE YOUR SCRIPT IS EXECUTABLE!!!!**

To make it executable, go to the file location in the terminal and run the following command

chmod +x *file name*


There is suppose to be a file called auto start in .config/lxession/LXDE-pi/autostart but all the times I've looked for it, it wasn't there so you can just create it

You can create it by running the following commands

#### First you can to be in the config directory, so from root run

cd .config/

#### Then make the lxsession directory

mkdir lxession

#### Then LXDE-pi

cd lxsession

mkdir LXDE-pi

#### Inside of LXDE-pi create the autostart file

cd LXDE-pi

touch autostart

#### You want to modify the autostart file, so run

sudo nano autostart

#### Inside the file add the following lines

@lxpanel --profile LXDE-pi

@pcmanfm --desktop --profile LXDE-pi

@lxterminal -e "file path to the script you want to run on start up"

@screensaver -no-splash

## Setting up the PI
Clone the repo in the desktop of the PI

Once you cloned it, navigate to the network tables directory

Make sure the install shell script is executable, to check run the command *ls -l* if it is green then it is executable

If it isn't green run *chmod +x install.sh*

Run the install shell script

*./install.sh*

After it is done reboot and the script should run on start up

