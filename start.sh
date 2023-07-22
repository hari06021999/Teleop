1. touch start.sh
2. sudo pico start.sh

//insert content
#!/bin/bash
/usr/bin/python file path location
cd source devel/setup.bash
roslaunch pkg_name launchfilename.launch

3. chmod +x start.sh

4. touch stop.sh
5. sudo pico stop.sh

//insert content
#!/bin/bash
for KILLPID in `ps ax | grep 'MyScript` | awk `{print $1;}`'; do
kill -9 $KILLPID;
done

6. chmod +x stop.sh

//Service create
sudo pico /etc/systemd/system/name_script.service

//Insert content 
[Unit]
Description =NAME_SCRIPT service
After = network.target

[Service]
Type=simple
ExecStart = /bin/bash /file path of start.sh script
ExecStop = /bin/bash /file path of stop.sh script
Restart = always
RestartSec=5
TimeoutSec=60
RuntimeMaxSec=infinity
PIDFile=/tmp/name_Script.pid

[Install]
WantedBy=multi-user.target

7. sudo systemctl enable /etc/systemd/system/name_script.service
8. sudo systemctl daemon-reload
9. sudo service name_script start
10. sudo service name_script status
11. sudo service name_script stop

12. sudo crontab -e
13. * * * * * sudo service name_script start


