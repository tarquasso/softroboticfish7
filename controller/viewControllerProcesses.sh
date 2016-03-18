echo "Controller py and sh processes:"
sudo ps -ef | grep controller.*.py | grep -v "grep" | grep -v "viewFishProcesses"
sudo ps -ef | grep controller.*.sh | grep -v "grep" | grep -v "viewFishProcesses"
