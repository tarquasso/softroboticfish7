dev="/dev/sda"
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
