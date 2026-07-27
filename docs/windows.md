# Windows status

Windows is not an acceptance platform. The portable core and CLI fallback
paths can be configured with:

```powershell
.\scripts\build_windows.ps1
```

The `windows-msvc` preset disables librealsense and OpenVINS. A future
best-effort native build can use vcpkg for OpenCV, Eigen, Boost, and
librealsense, but OpenVINS v2.7 and Ceres 2.1 compiler compatibility must be
verified independently. Do not introduce ROS or change the supported Ubuntu
implementation to make Windows work.

No native Windows D435i/OpenVINS claim is made until that exact combination is
built and tested with hardware.
