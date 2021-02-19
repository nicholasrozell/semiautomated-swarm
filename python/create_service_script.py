#!/usr/bin/env python3
import os

username = os.environ['HOME'].split('/')[-1]
path=os.getcwd()
proj_dir = os.path.dirname(path)

mavproxypix_file = '../services/Mavproxy_Pix.service'

datapix =f"""[Unit]
Description=MavproxyPix
After=network.target

[Service]
ExecStart={proj_dir}/scripts/mavproxy_script1.sh
WorkingDirectory={proj_dir}/scripts
StandardOutput=inherit
StandardError=inherit
Restart=always
User={username}

[Install]
WantedBy=multi-user.target
"""
f = open(mavproxypix_file, 'w+')
f.write(datapix)
f.close()

print(f"Created a service file named {mavproxypix_file}")

mavproxycomm_file = '../services/Mavproxy_Comm.service'
datacomm =f"""[Unit]
Description=MavproxyComm
After=network.target

[Service]
ExecStart={proj_dir}/scripts/mavproxy_script2.sh
WorkingDirectory={proj_dir}/scripts
StandardOutput=inherit
StandardError=inherit
Restart=always
User={username}

[Install]
WantedBy=multi-user.target
"""
f = open(mavproxycomm_file, 'w+')
f.write(datacomm)
f.close()

print(f"Created a service file named {mavproxycomm_file}")

mavros_file = '../services/MavROS.service'
datamavros =f"""[Unit]
Description=MavproxyComm
After=network.target

[Service]
ExecStart={proj_dir}/scripts/launch_mavros.sh %I
WorkingDirectory={proj_dir}/scripts
StandardOutput=inherit
StandardError=inherit
Restart=always
User={username}

[Install]
WantedBy=multi-user.target
"""
f = open(mavros_file, 'w+')
f.write(datamavros)
f.close()

print(f"Created a service file named {mavros_file}")
print("Created all service files")

'''
These files need to be copied to /etc/systemd/system folder and started
TODO
'''
