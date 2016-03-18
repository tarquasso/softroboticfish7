echo "-- Killing py and sh fish processes"

sudo ps -ef | grep fish.*.py | grep -v "grep" | grep -v "killFishProcesses" | awk '{print $2}' | xargs sudo kill &> /dev/null
sudo ps -ef | grep fish.*.sh | grep -v "grep" | grep -v "killFishProcesses" | awk '{print $2}' | xargs sudo kill &> /dev/null

