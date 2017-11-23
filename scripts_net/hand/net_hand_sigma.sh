#!/bin/sh
# Laurent LEQUIEVRE
# laurent.lequievre@univ-bpclermont.fr
# UMR 6602 - Institut Pascal

# Ip address of Pan Tilt -> 192.168.100.252

# NETWORK INFORMATIONS
# ====================
# sudo lshw -C network
# eth0 -> NetXtreme BCM5722 Gigabit Ethernet PCI Express, pci@0000:04:00.0, 00:0a:f7:93:2f:fa, tg3
# eth2 -> RTL8111/8168/8411 PCI Express Gigabit Ethernet Controller, pci@0000:05:00.0, 10:fe:ed:08:18:dc, r8169
# eth1 -> Ethernet Connection (2) I219-LM, pci@0000:00:1f.6, 48:4d:7e:ac:d2:ae, e1000e

# eth2 -> connexion shadow hand
# eth1 -> On board connexion internet
# eth0 -> connexion switch

# Table de routage actuelle
# route -n

# Associate ethX
# eth0 -> switch

ethX_associate_switch="eth0"
ethX_associate_fsensor="eth?"

# Define IP address for ethX associate to switch

ip_address_associate_switch="192.168.100.113"
ip_address_associate_sub_net_switch="192.168.100.0/24"
ip_address_associate_user_computer="192.168.100.105"
ip_address_associate_fsensor="192.168.1.100"

ip_address_fsensor="192.168.1.1"

# Define Netmask for switch card

netmask_associate_switch="255.255.255.0"
netmask_associate_user_computer="255.255.255.0"
netmask_associate_fsensor="255.255.255.255"

# Define hw of each ethX associated

hw_ethX_associate_switch="00:0a:f7:93:2f:fa"
hw_ethX_associate_fsensor="???????"

# path of ax88179_178a linux module
path_of_ax88179_178a="/home/kmohyeldin/git_projects/AX88179_178A/ax88179_178a.ko"


do_start()
{
	echo "load usb and ax88179_178a linux modules !"
	sudo modprobe usbnet
	sleep 3

	sudo insmod $path_of_ax88179_178a
	sleep 3
	
	echo "Start net config for switch !"

	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down
	
	echo "ifconfig down ethX associated to fsensor !"
	sudo ifconfig $ethX_associate_fsensor down


	echo "Start ethX associate to switch !"
	sudo ifconfig $ethX_associate_switch up $ip_address_associate_switch netmask $netmask_associate_switch hw ether $hw_ethX_associate_switch

    echo "Start ethX associate to fsensor !"
	sudo ifconfig $ethX_associate_fsensor up $ip_address_associate_fsensor netmask $netmask_associate_fsensor hw ether $hw_ethX_associate_fsensor   

	echo "Delete all route of eth0"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch


	echo "Add sub network for switch"
	sleep 5
	sudo route add -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch
	
	echo "Add route to fsensor"
	sleep 5
	sudo route add -host $ip_address_fsensor dev $ethX_associate_fsensor
}

do_stop()
{
 	echo "unload usb and ax88179_178a linux modules !"
 	sudo rmmod ax88179_178a
	sleep 3

 	sudo rmmod usbnet
	sleep 3

	echo "ifconfig down ethX associated to switch !"
	sudo ifconfig $ethX_associate_switch down

	echo "ifconfig down (non RT) ethX associated to fsensor !"
	sudo ifconfig $ethX_associate_fsensor down

	echo "Delete all routes of ethX associate to switch !"
	sleep 5
	sudo route del -net $ip_address_associate_sub_net_switch dev $ethX_associate_switch

	echo "Delete all routes of ethX associate to fsensor !"
	sleep 5
	sudo route del -host $ip_address_fsensor dev $ethX_associate_fsensor
}

case "$1" in

   start)

      do_start

      ;;

   stop)

      do_stop

      ;;

   *)

      echo "--> Usage: $0 {start|stop}"

      exit 1

esac

exit 0

