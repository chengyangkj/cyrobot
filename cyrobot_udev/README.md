# lino_udev

udev rule creator for cyrobot. 
## Creating udev rule
1. Run the udev creator:
```
$ rosrun lino_udev lino_udev.py
```

2. Copy the created udev rule to /etc/udev/rules.d/58-lino.rules:
```
$ sudo cp 58-lino.rules /etc/udev/rules.d/58-lino.rules
```