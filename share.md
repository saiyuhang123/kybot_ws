dia@nvidia-desktop:~/kybot_ws$ export LD_LIBRARY_PATH=/home/nvidia/kybot_ws/src/HK/C++demo/consoleDemo/linux64/lib:$LD_LIBRARY_PATH
nvidia@nvidia-desktop:~/kybot_ws$ ros2 run hk_camera hk_camera_gui
hpr tls index{13}
[INFO] [1784108858.446822842] [hk_camera_node]: Params: ip=192.168.1.64:8000, channel=1
[INFO] [1784108858.459874430] [hk_camera_node]: HKCameraNode initialized (SDK V0.0.0.0)
loop[2] find 4 mac and 5 ip
[2026-07-15 17:47:44.820][INF] The COM:HCCoreBase ver is 6.1.4.15, 2020_03_05. Async:1.
[2026-07-15 17:47:44.820][INF] The COM:Core ver is 6.1.11.5, 2025_12_04. Async:1.
[2026-07-15 17:47:44.820][INF] This HCNetSDK ver is 6.1.11.5 Ver 2025_12_04.
[2026-07-15 17:47:44.820][INF] COM_Login dev 192.168.1.64:8000.
[2026-07-15 17:47:44.821][INF] dwTotalNum[2048]
[2026-07-15 17:47:44.821][INF] Private connect 192.168.1.64:8000 sock=166 this=0x1fc3d9c4 cmd=0x10000 port=42194
[2026-07-15 17:47:44.821][INF] LogonDev1 in[192.168.1.64:8000]
[2026-07-15 17:47:44.823][ERR] PRO_RecvProData_NewMemory pRecv->uiDvrStatus[1300] is not QULIFIED(1)
[2026-07-15 17:47:44.872][ERR] PRO_RecvProData_NewMemory pRecv->uiDvrStatus[16777315] is not QULIFIED(1)
[Decoder-HW] Trying GStreamer hardware decode...
[Decoder-HW] Pipeline: uridecodebin uri=rtsp://admin:a1234567@192.168.1.64:554/h264/ch1/main/av_stream ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1 max-buffers=2
Opening in BLOCKING MODE 
NvMMLiteOpen : Block : BlockType = 279 
NvMMLiteBlockCreate : Block : BlockType = 279 
[ WARN:0@8.171] global cap_gstreamer.cpp:1750 open OpenCV | GStreamer warning: frame count is estimated by duration and fps
[Decoder-HW] Hardware decode OK (nvv4l2decoder)
[Decoder-HW] frame=1, size=1920x1080
[Decoder-HW] frame=30, size=1920x1080
[Decoder-HW] loop ended, frames=36
[Decoder-HW] Trying GStreamer hardware decode...
[Decoder-HW] Pipeline: uridecodebin uri=rtsp://admin:a1234567@192.168.1.64:554/h264/ch1/main/av_stream ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink drop=1 max-buffers=2
Opening in BLOCKING MODE 
NvMMLiteOpen : Block : BlockType = 279 
NvMMLiteBlockCreate : Block : BlockType = 279 
[ WARN:8@23.730] global cap_gstreamer.cpp:1750 open OpenCV | GStreamer warning: frame count is estimated by duration and fps
[Decoder-HW] Hardware decode OK (nvv4l2decoder)

