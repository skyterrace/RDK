RDK X5 连接蓝牙音箱
bluetoothctl默认镜像已经包含了的，可以直接进行蓝牙配对连接
$bluetoothctl 进入蓝牙控制台
  [bluetooth]# scan on
  [bluetooth]# pair <音箱MAC>
  [bluetooth]# trust <音箱MAC>
  [bluetooth]# connect <音箱MAC>
搜索蓝牙设备，找到MAC地址后进行配对并信任该设备，最后连接。
$bluetoothctl info <音箱MAC> 可以查看蓝牙音箱状态

要通过蓝牙音箱播放声音，还得安装和运行bluealsa服务
$sudo apt update
$sudo apt install bluez-alsa-utils
然后用which bluealsa看一下是否有/usr/bin/bluealsa
然后启动bluealsa服务
$sudo bluealsa -p a2dp-source -p a2dp-sink &

$bluealsa-aplay -L 应该能看到蓝牙音箱

接下来播放音乐
aplay -D bluealsa:<音箱MAC> k58im.wav
