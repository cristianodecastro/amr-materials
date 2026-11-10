# Connecting to the SkillsChallenge Robot

This guide is for a VMX robot that has already been configured and is running the SkillsChallenge robot program. It covers the normal Wi-Fi connection, an Ethernet recovery connection, Control Station, Shuffleboard, and optional VNC desktop access.

## What each connection does

- **Robot Wi-Fi or Ethernet** connects the student computer to the VMX network.
- **Studica Control Station** connects to the robot IP to show robot code/comms status and change mode or enable state.
- **Shuffleboard** connects to the robot's NetworkTables to display robot data.
- **VNC Viewer** provides a remote view of the Raspberry Pi desktop. It is optional and does not enable the robot or replace Control Station.

For this robot, the configured Wi-Fi robot address is **10.0.23.2** (team number 23). The direct Ethernet robot address is **172.22.11.2**. The Ethernet address is fixed and works even when the Wi-Fi password is unknown or Wi-Fi is not responding. Studica documents these VMX addresses and the team-number Wi-Fi address format in its [networking guide](https://docs.wsr.studica.com/en/latest/docs/VMX/networking.html).

## Normal connection over robot Wi-Fi

1. Power on the robot and wait for the VMX to finish starting. Keep the robot securely supported, with its wheels clear of the floor for initial checks.
2. On the student computer, open the Wi-Fi list and connect to the robot's configured access point. Use the SSID and Wi-Fi password provided by the instructor or shown on the robot's current setup card. Do not assume the factory SSID or password: this robot may have custom credentials.
3. Open a terminal on Windows and check reachability:

   ```powershell
   ping 10.0.23.2
   ```

   Replies show that the computer can reach the VMX. If ping times out, first confirm the PC is on the robot's Wi-Fi and that the robot has finished booting.

4. Open **Studica Control Station Console**. When it asks for the robot IP, enter:

   ```text
   10.0.23.2
   ```

5. Confirm **Robot Comms** and **Robot Code** are checked. The Joysticks indicator is expected to be checked only when the controller is connected to the computer and recognized by Windows. Studica's [Control Station instructions](https://docs.wsr.studica.com/en/latest/docs/ControlStation/Control%20Station%20Console/operation.html) describe the status indicators and controls.
6. Select **Teleoperated** mode with `o`. Leave the robot **Disabled** while checking the dashboard and controller inputs. Press `e` only when ready to move; press `d` to disable immediately.
7. Open Shuffleboard. The Control Station Console may start it automatically. If starting it separately, connect it to the same robot at `10.0.23.2` and open the **Teleop** tab. The active screen reports drive requests, wheel encoder counts and outputs, lift status, and servo controls.

## Ethernet recovery connection

Use this when the PC joins the robot Wi-Fi but Control Station cannot connect, the Wi-Fi password is unavailable, or Wi-Fi has stopped working. Ethernet connects directly to the VMX; it does not require the robot's Wi-Fi credentials.

1. Leave the robot powered on and **Disabled**. Close Control Station while changing network adapters so it does not keep trying the wrong address.
2. Connect an Ethernet cable directly between the PC and the VMX/Raspberry Pi Ethernet port.
3. Give the PC Ethernet adapter a static IPv4 address:
   - Open **Settings > Network & Internet > Advanced network settings > More network adapter options**.
   - Right-click the Ethernet adapter > **Properties** > **Internet Protocol Version 4 (TCP/IPv4)** > **Properties**.
   - Select **Use the following IP address** and enter:

     | Setting | Value |
     |---|---|
     | IP address | `172.22.11.1` |
     | Subnet mask | `255.255.255.0` |
     | Default gateway | Leave blank |
     | DNS servers | Leave blank |

   - Apply the settings. If Windows asks whether to trust the network, choose the option appropriate for a direct, private robot connection.

4. Check the link to the robot:

   ```powershell
   ping 172.22.11.2
   ```

5. Reopen Control Station and enter **172.22.11.2**. Confirm **Robot Comms** and **Robot Code** are checked, then choose Teleoperated with `o`. Keep it disabled with `d` until ready.
6. Connect Shuffleboard to **172.22.11.2** if it did not open automatically. Open the **Teleop** tab and check live values.
7. When finished, disconnect the cable and return the PC Ethernet adapter to **Obtain an IP address automatically** and **Obtain DNS server address automatically**. Reconnect to the robot Wi-Fi when needed.

If ping fails over Ethernet, check that the cable is seated at both ends and the PC static address is still `172.22.11.1/24`. Try another known-good cable. The VMX Ethernet address remains `172.22.11.2` regardless of the Wi-Fi team number.

## Optional remote desktop with RealVNC Viewer

Use VNC when you need to see or operate the VMX desktop, for example to inspect its desktop networking settings. It is not required for Control Station or Shuffleboard.

1. Install **RealVNC Viewer** on the student PC. Viewer is the client application; the VMX must already have VNC Server enabled and running.
2. First establish Wi-Fi or Ethernet as described above. In VNC Viewer, connect to the robot's reachable address: `10.0.23.2` over Wi-Fi or `172.22.11.2` over direct Ethernet.
3. If Viewer warns that it cannot verify the identity of the device, continue only after confirming the address is the robot you intend to access.
4. Sign in with the VMX desktop username and password supplied by the instructor. These credentials are separate from the Wi-Fi password. They may have been changed from the factory defaults, so do not assume defaults will work.
5. If VNC reports that the desktop cannot be shown, the VMX may be set to boot to a text console, or its VNC server/desktop service may not be enabled. Use a keyboard, display, and mouse connected to the robot, or ask the instructor to restore the configured desktop/VNC service. Avoid changing boot, display, Wi-Fi, or password settings during a class session unless instructed; those changes can interrupt robot access.

Studica's [VMX networking guide](https://docs.wsr.studica.com/en/latest/docs/VMX/networking.html) describes VNC Viewer access, first-connection identity prompts, and desktop boot troubleshooting. The VMX credentials used in this robot's setup were changed, so use the current credentials distributed by the instructor.

## If Control Station opens but shows no robot connection

1. Verify that the PC is connected to the robot Wi-Fi or the direct Ethernet network, not another saved Wi-Fi network.
2. Verify the address with `ping` and enter the matching robot IP in Control Station:

   | PC connection | Robot address to enter |
   |---|---|
   | Robot Wi-Fi, team 23 | `10.0.23.2` |
   | Direct Ethernet | `172.22.11.2` |

3. Close and reopen Control Station after changing between Wi-Fi and Ethernet, then re-enter the correct address.
4. If ping works but **Robot Comms** remains unchecked, disable Windows VPN software temporarily if permitted, check Windows Firewall's prompt for Control Station, and confirm no other computer is controlling the robot. Do not enable the robot while diagnosing a connection.
5. If **Robot Comms** is checked but **Robot Code** is not, the network path is working but the robot program is not reporting as running. Ask the instructor before restarting the robot or deploying code.
6. If both indicators are checked but Shuffleboard is blank, check its connection target and select the **Teleop** tab.

## Safe first check

Keep the robot's wheels off the ground. Connect, confirm status indicators, select Teleoperated mode, and inspect Shuffleboard while disabled. Check controller input indicators before enabling. When ready, enable with `e`, test one control briefly, and disable with `d` before touching or repositioning the robot. Autonomous mode is disabled in this project.
