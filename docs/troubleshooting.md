# Troubleshooting

If there are issues with the deploy script, make sure all of the following are true:

- You are using the Control Station IP as the first argument of the deploy script.
- You have made a clean build by entering `rm -rf build install log` into the terminal.
- You are on the main branch (There is no guarantee for experimental branches for complete functionality).
- The Control Station is using the correct Jetson IP.
- The robot is not EStopped
- Serial Node has `TESTING = False` in the **node.py** file found in the src/serial_node/serial_node directory.