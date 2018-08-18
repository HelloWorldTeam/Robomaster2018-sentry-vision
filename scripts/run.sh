#exec 2> /tmp/startup.log
#exec 1>&2
set -x

cd $(cd `dirname $0`; pwd)/../build
sudo sh /home/nvidia/jetson_clocks.sh
sudo sh ../scripts/load_can.sh 0
sudo ./main
