sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 15
sudo ip link set can0 up
# sudo ip link set can1 type can bitrate 1000000
# sudo ip link set can1 txqueuelen 15
# sudo ip link set can1 up
# right->can0 left->can1
