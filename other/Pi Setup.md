# Initialising Operating system

Setup SD card
1. Buy an SD Card and an SD card Reader
2. Boot up a computer with a SD card reader available slot
3. Connect the purchased SD card to the computer via the SD card reader
4. Download Raspberry Pi Imager on your computer. You can install the latest version from raspberrypi.com/software and run the installer.
5. Once you’ve installed Imager, launch the application by clicking the Raspberry Pi Imager icon or running rpi-imager
6. Click Choose device and select your Raspberry Pi model from the list.
7. Next, click Choose OS and select an operating system to install. In this case, I used Raspberry OS (64-bit)
8. Connect your preferred storage device to your computer and select it by clicking on "Choose Storage" on the menu and searching for it 
9. When prompted with OS customisation settings, click on "Edit settings"
The OS customisation menu lets you set up your Raspberry Pi before first boot. You can preconfigure:
A username and password
Wi-Fi credentials
the device hostname
The time zone
Your keyboard layout
Remote connectivity
When prompted for permission to load Wi-Fi credentials from your host computer. Click "Yes". Imager will prefill Wi-Fi credentials from the network you’re currently connected to.

10. When you’ve finished entering OS customisation settings, click Save to save your customisation.
11. Click Yes to apply OS customisation settings when you write the image to the storage device.
12. Respond Yes to the "Are you sure you want to continue?" popup to begin writing data to the storage device.
13. When you see the "Write Successful" popup, your image has been completely written and verified. You’re now ready to boot a Raspberry Pi from the storage device.

# Initialising Raspberry Pi
1. Slot the SD Card into the SD Card slot on the Raspberry Pi
2. Plug in peripherals, such as your mouse, keyboard, and monitor.
3. Plug in the power cable. You should see the status LED light up when your Pi powers on. If your Pi is connected to a display, you should see the boot screen within minutes.
4. If the Raspberry Pi shows a desktop page, it is ready for use.

# Initialising Raspberry Pi Connect
1. Open terminal on desktop
2. Run the following commands:
sudo apt update
sudo apt full-upgrade
sudo apt install rpi-connect
rpi-connect on (This allows Raspberry Pi Connect to be activated)
4. Click on the Connect plugin for the menu bar
5. Click Turn On Raspberry Pi Connect for the first time to open your browser, prompting you to sign in with your Raspberry Pi ID. Create one if you haven't.
6. After authenticating, assign a name to your device. Choose a name that uniquely identifies the device. Click the Create device and sign in button to continue.
7. You can now remotely connect to your device. The Connect icon in your menu bar will turn blue to indicate that your device is now signed in to the Connect service. You should receive an email notification indicating that a new device is linked to your Connect account.

Connect includes the ability to share your device’s screen in a browser. Use the following instructions to share your device’s screen.

1. Connect redirects you to the Raspberry Pi ID service to sign in. After signing in, Connect displays a list of linked devices. Devices available for screen sharing show a grey Screen sharing badge below the name of the device
2. Click the Connect via button to the right of the device you would like to access. Select the Screen sharing option from the menu. This opens a browser window that displays your device’s desktop.
3. You can now use your device as you would locally.
4. A green dot appears next to the Screen sharing badge in the Connect dashboard. This indicates an active screen sharing session
5. The Connect icon in the system tray turns purple and displays a closed circle when a screen sharing session is in progress. A desktop notification will appear whenever a screen sharing session starts.
6. To close a screen sharing session, click the Disconnect button above your desktop

# Using Python on Raspberry Pi
1. In the main menu (click the raspberry icon), click on programming and then Geany Programmer’s Editor.
2. Open a python file from the menu of geany
3. Start coding
