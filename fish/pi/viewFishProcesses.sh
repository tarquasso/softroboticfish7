echo "Fish py and sh processes:"
sudo ps -ef | grep fish.*.py | grep -v "grep" | grep -v "viewFishProcesses"
sudo ps -ef | grep fish.*.sh | grep -v "grep" | grep -v "viewFishProcesses"
