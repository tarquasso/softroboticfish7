echo "-- Killing py and sh controller processes"

sudo ps -ef | grep controller.*.py | grep -v "grep" | grep -v "killFishProcesses" | awk '{print $2}' | xargs sudo kill &> /dev/null
sudo ps -ef | grep controller.*.sh | grep -v "grep" | grep -v "killFishProcesses" | awk '{print $2}' | xargs sudo kill &> /dev/null

