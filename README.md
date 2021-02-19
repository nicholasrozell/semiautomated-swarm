# semiautomated-swarm
Required:
1. Ubuntu 20.04 or above installed on singleboard computers(SBC) with ssh enabled
2. Windows computer connected to the SBCs through a network switch

Install instructions:
1. ssh into the SBCs from the windows machine and clone this repository
	$git clone https://github.com/nicholasrozell/semiautomated-swarm.git

2. cd into the scripts directory in the semiautomated-swarm folder and copy the install_script.sh file to home directory

3. provide permissions and run the script
	$chmod +x install_script.sh
	$./install_script.sh

4. This script will clone ardupilot and install the required libraries to run this project. Follow the instructions on the terminal and provide password when prompted
5. Repeat these steps for every SBC on the network
