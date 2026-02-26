#!/usr/bin/bash
sudo echo 'SUBSYSTEM=="usb", ATTRS{idProduct}=="f100", ATTRS{idVendor}=="fccf", GROUP="users", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/99-fccf-f100.rules
sudo udevadm control --reload
sudo udevadm trigger