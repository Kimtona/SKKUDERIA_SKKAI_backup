GUI applications in remote containers are a bit of a pain in the neck. Which makes GUI applications in remote VSCode devcontainers are even more of a pain.

## GUI applications in remote Docker containers
If you want to launch a GUI application from a remote docker container, as for example during [mapping](https://git.ee.ethz.ch/pbl/research/f1tenth2/race_stack/-/tree/master/base_system/pbl_f110_system?ref_type=heads#mapping), a couple of specific steps need to be taken. 

1. Connect to a car via SSH, enabling X forwarding with the `-X` flag: 
```bash
ssh -X <username>@<car_ip>
```

2.  Move to the ForzaETH race stack directory, and run the `xauth_setup.sh` script:
```bash
cd <racestack_directory>
source .devcontainer/.install_utils/xauth_setup.sh
```
You should get at least the first line of the following output:
```
non-network local connections being added to access control list
xhost:  must be on local machine to add or remove hosts.
```

3. run the container with the appropriate script. 

4. Enjoy a terminal with GUI forwarding!

## GUI applications in remote VSCode devcontainers
Due to the complicatedness of how a VSCode container is spun up, connecting to GUI applications requires a bit more involvement and a secondary SSH connection, to which we can relay the X forwarding.
Due to this reason, both a terminal **and** VSCode need to be opened in the car. 

1. Connect to a car via SSH, enabling X forwarding with the `-X` flag: 
```bash
ssh -X <username>@<car_ip>
```

2.  Move to the ForzaETH race stack directory, and run the `xauth_setup.sh` script:
```bash
cd <racestack_directory>
source .devcontainer/.install_utils/xauth_setup.sh
```
You should get at least the first line of the following output:
```
non-network local connections being added to access control list
xhost:  must be on local machine to add or remove hosts.
```

3. Memorize the `DISPLAY` number in this SSH-connected terminal, after printing it to screen:
```bash
echo $DISPLAY
```

an example output could be 
```
localhost:10.0
```


4. open up the devcontainer on the car, first by opening up VSCode, then connecting to the car with the remote connection button to the bottom left (Connect to Host...), then open the race stack folder, and reopen in the devcontainer.

5. In the devcontainer terminal where you want to use the GUI application, export now the `DISPLAY` variable number. For example:
```bash
export DISPLAY=localhost:10.0
```
**Note**: use the full name as from the output of point 3.
 
6. Enjoy a terminal with GUI forwarding!

## Mac Support
XQuartz can display ordinary X11 applications, but its GLX implementation does
not provide the OpenGL context required by RViz 2. Use the official
[ETH-PBL remote-novnc](https://github.com/ETH-PBL/remote-novnc) workflow
instead. It runs a Linux Xvfb display and exposes it through noVNC.

Clone and set up the helper next to this repository:

```bash
cd ..
git clone https://github.com/ETH-PBL/remote-novnc.git
cd remote-novnc
./setup_vnc.sh
```

Start it before the race-stack DevContainer:

```bash
cd ../remote-novnc
./start_vnc.sh
```

Keep that terminal open and open the noVNC URL printed by the script. On
macOS, `start_vnc.sh` starts its own Docker container. Its X display number and
published ports are derived from the macOS user ID. For example, UID 501 uses
X display `:501`, X11 TCP port 6501, VNC port 10501, and noVNC port 20501.

The `arm` service in `docker-compose.yaml` connects to that display through
`host.docker.internal:${HOST_UID}` and forces Mesa software rendering
(LLVMpipe). No XQuartz `DISPLAY` export is needed.
