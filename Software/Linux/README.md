# Earth Rover Mini+ Host Development

## Build Example
```
cd earth-rover-mini/Software/Linux
git submodule update --init
mkdir build
cd build
cmake ..
make
```

## Connect to Robot
- Install ADB on your development platform
- Search and connect the wireless network <b>frodobot_xxx</b>. The default password is <b>12345678</b>.
- Once the connection is established. Connect to your robot with adb
```bash
adb connect 192.168.11.1
adb push hello /data/
adb shell
./hello
```
