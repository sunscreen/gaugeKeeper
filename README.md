# gaugeKeeper
New Pican version.

root@raspberrypi:/home/pi# ls -al /etc/systemd/network/
total 16
drwxr-xr-x 2 root root 4096 Sep 15 11:25 .
drwxr-xr-x 5 root root 4096 Sep 11 20:23 ..
-rw-r--r-- 1 root root   54 Sep 15 11:23 80-can.network
-rw-r--r-- 1 root root   53 Sep 15 11:25 81-can.network
lrwxrwxrwx 1 root root    9 May  7 15:42 99-default.link -> /dev/null
root@raspberrypi:/home/pi#




root@raspberrypi:/home/pi# cat /etc/systemd/network/80-can.network
[Match]
Name=can0
[CAN]
BitRate=500K
RestartSec=100ms

root@raspberrypi:/home/pi# cat /etc/systemd/network/81-can.network
[Match]
Name=can1
[CAN]
BitRate=500K
RestartSec=100msroot@raspberrypi:/home/pi#



root@raspberrypi:/home/pi# cat /etc/systemd/system/canfilter.service
[Unit]
Description=BMW CAN FILTER
Documentation=none available
After=network.target auditd.service

[Service]
ExecStart=/home/pi/canfilter
ExecReload=/bin/kill -HUP $MAINPID
KillMode=process
Restart=on-failure
RestartPreventExitStatus=255
Type=idle
RuntimeDirectory=/home/pi

[Install]
WantedBy=multi-user.target
Alias=canfilter.service






