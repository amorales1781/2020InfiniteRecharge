#!/bin/bash

sudo apt-get update
sudo apt install -y libjasper1 libjasper-dev libqtgui4 libqt4-test libatlas3-base libhdf5-dev
sudo apt install -y python3-pip
pip3 install opencv-contrib-python==4.1.0.25
pip3 install pynetworktables
mkdir ~/.config/lxsession/
mkdir ~/.config/lxsession/LXDE-pi/
cp autostart ~/.config/lxsession/LXDE-pi/
chmod +x NetworkTables.py
