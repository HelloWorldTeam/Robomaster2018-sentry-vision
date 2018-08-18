# 2018-04-30 by LSS

echo can $1 init...
modprobe can
modprobe can_raw
modprobe mttcan
ip link set can$1 type can bitrate 1000000
ip link set up can$1
echo can $1 ready
