# Installation Guide for Latte Panda

### Pre-installation

Test internet connection with.
> ping archlinux.org

To setup wifi use.
> iwctl

The iwd prompt should start.
> device = The device wanted to connect to the internet, i.e. wlan0
>
> SSID = Service Set Identifier, i.e. the networks name

Use the commands as follow to connect wirelessly.
Scans for connectable networks.
> station "device" scan

Lists the scanned networks.
> station "device" get-networks

To connect to a network use.
> station device connect SSID

A prompt should ask for the network's password.
This could take a few attempts to connect properly as an error will be printed if the computer
was not able to connect.
 
Once the computer is connect properly reuse the ping command to test it.

To update the system clock to ensure accuary use.
> timedatectl set-ntp true

To check the service status use the following command.
> timedatectl status

To list disks that are visible use.
> fdisk -l

To partition a disk the name must be visible .
For the Latte Panda the disk is title mm0blk0.
Use the command to start partitioning the chosen disk.
partition = mm0blk0

> fdisk /dev/partition

The fdisk prompt should start, use command shown for help.
To start partitioning the disk use command.
> g

To start a GPT table

To make a new partition use command.
> n

The first partition will be by defualt 1. Each partition also by defualt will be called a 'Linux Filesystem' this will be covered later.
The beginning of the partition can be skipped, press 'Enter' to skip.
For the end of the partition enter.
> +512M

To create a partion of 512 megabytes.
This will be the boot partion.

Press 'n' again, and '2'.
Skip the beginning of the partition.
For the end, enter.
> +10G

This is the swap partition that is 10 gigabytes.

Press 'n' again, and '3'.
Skip the beginning of the partition.
For the end, this can be skipped as well to use the rest of the storage space. Press 'Enter.'

To recognize each partition for their proper use, Press 't' and choose '1.'
Press L to list all the partition types. Use ':q' to quit out of the list.
For partition 1 it will be of a EFI partition. Press '1'.
For 2 it will be a Linux Swap.
Partition 3 can be left as is.

To write and quit out of fdisk, press 'w'.

To format these parititons for their proper use. 
Use the command for partition 1 (mm0blk0p1).
> mkfs.fat -F32 /dev/boot_partition 

For partition 2 (mm0blk0p2).
> mkswap /dev/swap_partiiton 

To turn on the swap_partition use the command.
> swapon /dev/swap_partition

For partition 3 (mm0blk0p3).
> mkfs.ext4 /dev/root_partition 

### Installation

To start installing Arch Linux a parition must be mounted use the root partition.
root_partition = mm0blk0p3
> mount /dev/root_partition /mnt

To install the base package and linux kernal, headers and firmware use.
> pacstrap /mnt base linux-lts linux-lts-headers linux-firmware

### Configuring the System

To generate the system's file system use.
> genfstab -U /mnt >> /mnt/ect/fstab

To start configuring use the command.
> arch-chroot /mnt

Set the timezone with the commands.
> ln -sf /usr/share/zoneinfo/America/Chicago /etc/localtime
>
> hwclock --systohc

Next install a text edition of choice, i.e. nano, vim. Use
text_editor = nano, vim, ...
> pacman -S text_editior

Uncomment any needed locales, by editing locale.gen file.
We want en_US.UTF-8.
> nano /etc/locale.gen

Find the line 'en_US.UTF-8 UTF-8.'

To generate the locales use the command.
> locale-gen

Create the locale.cof file and set the LANG variable.
> nano /etc/locale.conf
>> LANG=en_US.UTF.8

Create the hostname file.
> nano /etc/hostname
>> myhostname 

Add matching entries to the host file.
> nano /etc/hosts
>> 127.0.0.1    localhost
>>
>> ::1          localhost
>>
>> 127.0.1.1    myhostname.localdomain myhostname

Set the root password with
> passwd

This should not be something easy like 'admin'.

To install a boot loader use.
> pacman -S grub

For booting a manager is necessary.
> pacman -S efibootmgr

Make a directory for the boot directory.
> mkdir /boot/EFI

Mount the boot parition.
boot_partition = mm0blk0p1
> mount /dev/boot_partition /boot/EFI

Use the following command to install grub.
> grub-install --target=x86_64-efi --bootloader-id=grub_uefi --recheck

For stability install a microcode package.
> pacman -S intel-ucode

Create a grub config file.
> grub-mkconfig -o /boot/grub/grub.cfg

### Adding Users

To delegate system administrator.
> pacman -S sudo

To add a user use the command.
> useradd -m myuser

To change the password for the new user.
> passwd myuser

To add privaliges to the new user.
> usermod -aG wheel,optical,storage,uucp myuser

Additionally a line needs to be uncommented in visudo.
> EDITOR=text_editor visudo

Uncomment the line %wheel ALL=(ALL)=ALL. Keep the percent sign.

### Additional packages

> pacman -S base-devel git cmake gcc python python-pip wget networkmanager openssh neofetch bash-completion

Enable both NetworkManager and openssh use the command.
> systemctl enable NetworkManager sshd

Once the steps are complete unmount from the systems using.
> umount -l /dev/root_partition

Now shutdown the computer using.
> shutdown now

Unmount the boot drive for the computer and turn it back on, you should be greeted by a grub menu listing Arch Linux kernals.

Sign in using the user created earlier.

To list nearby wifi networks use.
> nmcli device wifi list

To connect to a network use the commands.
> nmcli device wifi connect SSID_or_BSSID password password

An AUR helper is an option to manage AUR packages easier.
To install yay the run the following commands.
> git clone https://aur.archlinux.org/yay.git
>
> sudo mv yay /opt
>
> cd /opt/yay
>
> makepkg -si

To install Ros we first want the geographiclib libraries.
Use the command to install the package.
> yay -S geographiclib

Then to install the datasets use.
> geographiclib-get-geoids best

For ROS installations use the command
> yay -S ros-noetic-ros-base ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-mavros-extras

Once this is completed (will most likely take 2+ hours) use the commands to initialize ROS.
> sudo rosdep init
>
> rosdep update

To setup the ROS enviroment use.
> echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
>
> source ~/.bashrc

Then install MavProxy using
> pip install MAVProxy

To hide the grub menu on boot write the following line in /etc/defual/grub
> GRUB_FORCE_HIDDEN_MENU="true"

Now create the file /etc/grub.d/31_hold_shift containing this [script](https://gist.githubusercontent.com/anonymous/8eb2019db2e278ba99be/raw/257f15100fd46aeeb8e33a7629b209d0a14b9975/gistfile1.sh)

Now make the file and executable and regnerate the grub configurations
> chmod a+x /etc/grub.d/31_hold_shift
>
> grub-mkconfig -o /boot/grub/grub.cfg

Then reboot the system for it to take effect.

### Links:

[Arch Linux Website](https://archlinux.org/)

[Arch Linux Documentation](https://wiki.archlinux.org/)

[Arch Linux Download](http://mirrors.mit.edu/archlinux/iso/2021.05.01/)
