# Newton Ice Shaver Controller Setup

Follow the steps below on a fresh Raspberry Pi 4 (Bullseye/Bookworm).

1. **Update the system**
   ```bash
   sudo apt update
   sudo apt full-upgrade -y
   ```

2. **Install system packages**
   ```bash
   sudo apt install -y python3 python3-pip python3-pyqt5 python3-pyqt5.qtquick pigpio python3-pigpio python3-rpi.gpio git
   ```

3. **Enable the pigpio daemon** (required for PWM control)
   ```bash
   sudo systemctl enable pigpiod
   sudo systemctl start pigpiod
   ```

4. **Install Python dependencies** (creates a virtual environment to keep things tidy)
   ```bash
   python3 -m venv ~/ice_shaver_env
   source ~/ice_shaver_env/bin/activate
   pip install --upgrade pip
   pip install pigpio RPi.GPIO PyQt5
   ```

5. **Run the Newton controller UI**
   ```bash
   cd ~/Newton
   source ~/ice_shaver_env/bin/activate
   python3 -m simple_control.ui_simple
   ```

6. **Autostart on boot (optional)**
   Create `/etc/systemd/system/ice-shaver-ui.service` with the following:
   ```ini
   [Unit]
   Description=Newton Ice Shaver RPM Controller
   After=network.target pigpiod.service

   [Service]
   Type=simple
   User=pi
   WorkingDirectory=/home/pi/Newton
   Environment="DISPLAY=:0"
   ExecStart=/home/pi/ice_shaver_env/bin/python3 -m simple_control.ui_simple
   Restart=on-failure

   [Install]
   WantedBy=graphical.target
   ```

   Enable the service:
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable ice-shaver-ui.service
   sudo systemctl start ice-shaver-ui.service
   ```

7. **Shutdown safety**
   Always stop the UI before powering off the PI to guarantee the motor driver is disabled cleanly.
