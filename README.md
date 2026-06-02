# Limb-Graphical-User-Interface-App
3. Setup Instructions
3.1. EMG System
3.1.1 EMG Electrode Connections:
The Limb++ base packet includes 11 EMG electrodes. The included electrodes are not reusable. The Limb++ system is compatible with most EMG/ECG electrodes available on the market. We recommend 3M’s Red Dot ECG electrodes.

One electrode is to be connected to the elbow.

Two electrodes are to be connected to the deltoid (upper arm shoulder area).

Two electrodes are to be connected to biceps (upper arm forward area)

Two electrodes are to be connected to the triceps (upper arm back area)

Two electrodes are to be connected to flexor digitorum (lower arm outer side area)

Two electrodes are to be connected to pectoralis major (front chest area)


The electrodes can be placed on either side of the body. If placement on either arm is not possible, please request your doctor to contact us for a personalized solution to be engineered.

After the electrode placement, place the “Limb++ Circuit Set” close to you, preferably on a flat surface such as a table.

There are 11 cables coming out of the circuit set; 5 pairs are labeled as “Hand”, “Biceps”, “Triceps”, “Deltoid”, and “Chest”; connect each pair to its corresponding electrode pair. A cable pair related to a muscle group can be connected to its corresponding electrode pair in either way, meaning that there are 2 equally valid connection options for each muscle group.

Then, connect the single cable labeled “Ref” to the elbow electrode.

Please make sure that the cables are stable. For better performance, keep the cables extended, do not roll them up into a pile. Try to keep other electronic devices and power connections away from the system so as to not cause interference.
3.1.2 EMG System Power Connections:
The “Limb++ Circuit Set” uses 6 x 9V batteries.

Please open the “Limb++ Circuit Set” and place 9V batteries in the battery slots. Please make sure that the batteries are at least 75% charged for proper operation. OTAVYA recommends using Duracell batteries.
3.2. Limb++ GUI App
3.2.1 Installation:
The Limb++ GUI App is an open-source software, and can be downloaded from https://github.com/OTAVYA/Limb-Graphical-User-Interface-App by pressing the “Code” button and pressing “download .ZIP file”. After the completion of the installation, unzip the folder.

OTAVYA is not responsible for any harm that may be caused by modified versions of the software.
3.2.2 Minimum System Recommendations:
OS: Windows 11
System Type: x64
Processor: Intel i7-11800H
RAM: 16GB
GPU: Nvidia GeForce RTX 3060
3.2.3 Launch:
The Limb++ GUI App requires a Python interpreter installed on your personal computer. To launch the app, navigate to the system location of the downloaded folder. Inside the folder, execute the following command in terminal:
python limb_pp_gui.txt
This will launch the app.

3.2.3 GUI Guide:
3.2.3.1. GUI Overview

3.2.3.2. Mode Selection
The Limb++ GUI App has 3 modes for robot control:

Full-Arm EMG Control: This is the main control mode to be used with EMG sensor data. Requires the Limb++ EMG Signal Acquisition Circuit Set to be powered on. Gripper twist is disabled in this mode.


Keyboard Control: The robot can be controlled with keyboard inputs.


Position Control: The robot can be given a target pose which it will slowly approach.

3.2.3.3. Calibration
If previously recorded calibration data is to be used, please press the “Use existing calibration” button.

If new calibration is to be made, please press the “New Calibration” button. The user is given a series of instructions which can be read on the screen and heard audibly narrated. Please follow the instructions closely, and observe the EMG signal levels on the screen to see that your actions are being represented in a responsive manner.

If the calibration results are unsatisfactory for one or more muscle groups, individual muscle groups’ recalibration buttons can be pressed for a short recalibration.

3.2.3.4. Full-Arm EMG Control
The actions and their corresponding system responses are as follows:

Relax

System keeps its position


Arm Flexation

The robotic arm’s horizontal distance between its end-effector and its base is decreased


Arm Extension

The robotic arm’s horizontal distance between its end-effector and its base is increased


Dropping Arm Down

The robotic arm end-effector height is decreased


Raising Arm Up

The robotic arm end-effector height is increased


Compressing Chest

The robotic arm changes its yaw angle. The change is in a flipping direction, meaning it changes in the opposite direction of its last change.


Opening Hand

Opens the gripper


Closing Hand

Closes the gripper
3.2.3.5. Keyboard Control
Holding down the buttons indicated on the screen moves the robotic arm in the desired location.
3.2.3.6. Position Control
The desired orientation values can be entered numerically, pressing “enter” starts the movement of the system to the desired orientation.
3.2.3.7. Test Mode (Advanced)
The “Start Test” button starts an EMG signal processing accuracy analysis test where the user is told to execute a number of physical movements. The test data is automatically saved as a .csv file, which can be analysed by a method of the user’s choice.
3.2.3.8. Settings
The settings include 4 menus:
“Sound” allows the manipulation of the narrator voice level
“Graphics” allows the user to turn on/off the robot arm visualization. If the app is running poorly, try turning this option off.
“Key Binding” allows the user to set which keyboard button corresponds to which command in the keyboard control mode.
“Advanced” allows the user to manipulate EMG signal processing parameters.

3.4. Limb++ Robotic Arm (Base Version)
3.4.1 Setup:
The base version of the robotic arm delivered includes an assembled robotic arm. The only mechanical setup step left for the user is to find a solid base for the robot. Based on the needs of the user, the base can be screwed to a table or a wooden base available by request.

3.4.2 Power & Data Connections:
The robotic arm set includes a 230V-7.2V power adapter. Please plug this adapter to a safe power outlet and connect it to the power connection extension of the robotic arm.

The USB-Micro USB cable found in the package must be connected between your personal computer and the microcontroller of the robot set.

	

3.4. Warnings
Only connect the EMG electrodes to a person after their doctor’s approval
Do not alter the recommended EMG connections without consulting your doctor
Do not spill water on any of the parts of the Limb++ system
Keep all system kits away from electronics that may cause signal interference
Make sure that the batteries used are charged at least 75% and the grid connection is stable
Keep the two separable parts of the system no more than 10m apart.
Make sure that the robotic arm is firmly attached to its base
Make sure to give the robotic arm a free-space  of at least 30cm radius
Do not attempt to lift objects weighing more than 60g or objects longer than 15cm with the robotic arm.
