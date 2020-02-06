Instructions for Linux:

1. [Download and build Unreal Engine from source.](https://docs.unrealengine.com/en-US/Platforms/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html)

2. Clone the 'UnrealF1Tenth' github repo (https://github.com/abol-karimi/UnrealF1Tenth.git).

3. Open the project in Unreal Editor. Click on the compile button on top of the viewport. Then press Play.

4. In a terminal, run `roslaunch rosbridge_server rosbridge_tcp.launch bson_only_mode:=True`.

5. Now you can control the car by publishing `std_msgs/Float32` numbers to `/commands/motor/duty_cycle` (speed) and `/commands/servo/position` (steering).
