# RTCAN_Motor
FT센서 바이어스 명령어 :
sudo ethercat download -p0 -t uint16 0x2001 1 1
FT센서 LowpassFilter Hz 명령어 :
cut-off frequency 1000Hz
sudo ethercat download -p0 -t uint16 0x2001 2 1000
cut-off frequency 100Hz
sudo ethercat download -p0 -t uint16 0x2001 2 100
cut-off frequency 10Hz
sudo ethercat download -p0 -t uint16 0x2001 2 10
