#!/usr/bin/env python
import glib
import gudev
import sys
import signal
import io
import os

def callback(client, action, device, user_data):
    vendor_name = device.get_property("ID_VENDOR_ENC")
    device_id = device.get_property("ID_SERIAL_SHORT")
    model_id = device.get_property("ID_MODEL_ID")
    vendor_id = device.get_property("ID_VENDOR_ID")
    if vendor_id == "10c4" or vendor_id == "067b":
        kernel = "ttyUSB?"
    else:    
        kernel = "ttyACM?"

    if action == "add":
        global udev_rules

        print vendor_name + " detected!"

        while True:
            reply = raw_input("Create uDev name? (Y/N) ")
            if reply.upper() == 'Y':
                sym_link = raw_input("What do you want to name this port? : ")
                udev_string = "KERNEL==\"%s\", SUBSYSTEM==\"tty\", ATTRS{idVendor}==\"%s\", ATTRS{idProduct}==\"%s\", ATTRS{serial}==\"%s\", MODE=\"0666\" SYMLINK+=\"%s\"\r\n" %(kernel, vendor_id, model_id, device_id, sym_link)
                udev_rules += udev_string
                break

            elif reply.upper() == 'N':
                break

            else:
                pass

        print "Plug in next device. Press CTRL+C to save your udev rules.\r\n"

if __name__ == "__main__":
    print "Plug in your usb device\r\n"
    client = gudev.Client(["usb/usb_device"])
    client.connect("uevent", callback, None)
    udev_rules=""

    try:
        loop = glib.MainLoop()
        loop.run()

    except KeyboardInterrupt:
        while True:
            save_rules = raw_input(" Do you wany to save your uDev_rules?(Y/N) : ")
            if save_rules.upper() == 'Y':
                if len(udev_rules) > 0:
                    with io.FileIO("58-lino.rules", "w") as file:
                        file.write(udev_rules)
                    print "\r\n58-lino.rules saved."
                    print "\r\nRUN: $ sudo cp 58-lino.rules /etc/udev/rules.d"
                    print "\r\nRestart the computer once done."
                    break
                else:
                    break

            elif save_rules.upper() == 'N':
                break
            else:
                pass
sys.exit()
