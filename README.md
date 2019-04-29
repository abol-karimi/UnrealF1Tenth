You need Unreal Engine and Visual Studio to run the code on Windows.

1. Make an account on unrealengine.com

2. Download the Unreal Engine installer for Windows. Execute the installer and login to your account. In (Epic Games Lanucher) click the "install" tab on the right top corner and install the engine version 4.21.2.

3. Run Unreal Engine.

4. Select the C++ tab and it will prompt you (at the bottom) to install Visual Studio 2017 (if not already installed).

5. Install Visual Studio 2017 with the components that Unreal engine already selected. If on Windows 10, make sure Windows 10 SDK is selected.

6. Clone the 'UnrealF1Tenth' github repo (https://github.com/abol-karimi/UnrealF1Tenth.git), or use the submitted code on Sakai.

7. Go to F1Tenth directory. Right click on F1Tenth.uproject and select "Generate Visual Studio project file". If there is no such option, first we need to make sure that ".uproject" files are associated with Unreal, as follows: Copy UnrealVersionSelector.exe from "C:\Program Files (x86)\Epic Games\Launcher\Engine\Binaries\Win64" to "C:\Program Files\Epic Games\UE_4.21\Engine\Binaries\Win64". Then run "UnrealVersionSelector.exe" from the latter folder and click Yes on prompt "Register this directory as an Unreal Engine installation?"

8. Open F1Tenth.uproject by double-clicking on it. This opens the project in Unreal Editor.

9. Click on the compile button on top of the viewport. Then click on the arrow next to Play button on the top of the viewport and select Simulate to run the simulation.

10. In Unreal Editor, you can select an F1TenthPawn in the WorldOutliner panel, or by clicking on the vehicle actor mesh in one of the viewports. Then in the details tab, you can select "AI Controller Class" to be either "DisparityAIController" or "VoronoiAIController". Also you can choose "Player 0" for "Auto Possess Player" to drive the car with keyboard.