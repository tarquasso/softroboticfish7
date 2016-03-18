echo "-- Killing fish processes"
sudo ps -ef | grep "fish/python" | awk '{print $2}' | xargs sudo kill
sudo ps -ef | grep "fish/mbed" | awk '{print $2}' | xargs sudo kill


