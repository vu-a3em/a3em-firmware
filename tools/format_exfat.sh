#!/bin/sh

if [ $# -ne 1 ]; then
	echo "Usage: ./format_exfat [VOLUME_MOUNT_POINT]"
	exit 1
elif [ ! -e "$1" ]; then
	echo "Mount Point $1 does not exist"
	exit 2
elif [ "${EUID:-$(id -u)}" -ne 0 ]; then
	echo "Script must be run as root"
	exit 3
fi

volume=${1%/}
if [ "$(uname -s)" = "Darwin" ]; then
	disk=$(diskutil list -plist | plutil -convert json -r -o - - | jq -c '[.AllDisksAndPartitions[], .AllDisksAndPartitions[].APFSVolumes, .AllDisksAndPartitions[].Partitions] | flatten | map(select(has("DeviceIdentifier") and has("MountPoint")))[] | {mount: .MountPoint?, id: .DeviceIdentifier?}' | jq -r --arg volname "$volume" 'select(.mount == $volname) | .id?')
	read -p "Format $disk for A3EM? [Y/N]: " confirm && [[ "$confirm" == [yY] ]] || exit 1
	diskutil unmountDisk $disk
	newfs_exfat -b 4096 -v A3EM $disk
	diskutil mountDisk $disk
else
	if ! dpkg -s util-linux >/dev/null 2>&1; then apt -y install util-linux >/dev/null 2>&1; fi
	if ! dpkg -s exfatprogs >/dev/null 2>&1 && ! apt -y install exfatprogs >/dev/null 2>&1; then
		if ! dpkg -s exfat-fuse >/dev/null 2>&1 || ! dpkg -s exfat-utils >/dev/null 2>&1; then
			apt -y install exfat-fuse exfat-utils >/dev/null 2>&1
		fi
	fi
	disk=$(mount | grep "on $volume" | cut -f 1 -d " ")
	pdisk=$(lsblk -ndo pkname $disk)
	if [ -z "$pdisk" ]; then pdisk=$disk; else pdisk="/dev/"$pdisk; fi
	read -p "Format $disk for A3EM? [Y/N]: " confirm && [ "$confirm" = "y" -o "$confirm" = "Y" ] || exit 1
	umount $disk
	wipefs --all --force $disk
	wipefs --all --force $pdisk
	len=$(expr $(fdisk -l $pdisk | grep -o bytes,.*sectors | cut -f 2 -d " ") - 600)
	dd if=/dev/zero of=$pdisk bs=512 count=600
	dd if=/dev/zero of=$pdisk bs=512 seek=$len
	(echo n; echo p; echo 1; echo 4096; echo " "; echo t; echo 07; echo w) | fdisk $pdisk
	mkfs -t exfat -c 4096 -L A3EM $disk
	mkdir -p $volume
	mount $disk $volume
fi
