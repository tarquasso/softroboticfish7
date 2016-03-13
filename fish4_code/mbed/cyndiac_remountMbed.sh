echo "-- Unmounting mbed"
sudo umount /media/cyndiac/MBED
sleep 2
echo "--- Removing as a file"
sudo rm /media/cyndiac/MBED*
echo "--- Removing as a directory"
sudo rmdir /media/cyndiac/MBED*


dev="/dev/sdb"
if [ "$#" -eq 1 ]; then
  dev=$1
fi

echo "-- Mounting mbed to " $dev
sudo mkdir /media/MBED
sleep 0.25
sudo mount $dev /media/MBED
sleep 5
echo "-- If you see mbed's files here then it worked!"
sudo ls /media/MBED
