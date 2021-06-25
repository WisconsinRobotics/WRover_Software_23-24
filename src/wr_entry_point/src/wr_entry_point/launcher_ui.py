import os
import re
from typing import Optional

from paramiko import SSHClient
from qt_gui.plugin import Plugin
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import QApplication, QCheckBox, QComboBox, QPushButton, QWidget
import rospkg

INET_ADDR_PATTERN = re.compile(r'inet (\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})/\d{1,2}')

def do_launch(launch_file: str, local_mode: bool, mock_mode: bool):
    
    if local_mode:
        os.environ['WROVER_LOCAL'] = 'true'
        os.environ['ROS_MASTER_URI'] = 'http://localhost:11311'
        os.environ['ROS_IP'] = '127.0.0.1'
    else:
        # launch roscore on the rover
        local_inet_addr: str
        with SSHClient() as ssh_cli:
            ssh_cli.load_system_host_keys()
            ssh_cli.connect('wrover-nano.local', username='wiscrobo')
            local_inet_addr = ssh_cli.get_transport().sock.getsockname()[0]
            
            # retrieve rover ethernet ip address
            _, ip_addr_stdout, _ = ssh_cli.exec_command('ip addr show eth0')
            rover_inet_addr: Optional[str] = None
            with ip_addr_stdout:
                for line in ip_addr_stdout:
                    match = INET_ADDR_PATTERN.search(line)
                    if match:
                        rover_inet_addr = match.group(1)
                        break
            if rover_inet_addr is None:
                raise ValueError('No eth0 internet address could be discovered on the rover!')
            
            # run roscore
            _, bootstrap_stdout, _ = ssh_cli.exec_command('~/catkin_ws/WRover21_Software/env_remote.sh ~/catkin_ws/WRover21_Software/bootstrap.sh',\
                environment={
                    'ROS_MASTER_URI': 'http://localhost:11311',
                    'ROS_IP': rover_inet_addr,
                    'ROS_HOSTNAME': 'wrover-nano.local'
                })
            with bootstrap_stdout:
                bootstrap_stdout.read()
            
            print(f'LOCAL ADDR: {local_inet_addr}')
            print(f'REMOTE ADDR: {rover_inet_addr}')
        
        os.environ['WROVER_LOCAL'] = 'false'
        os.environ['ROS_MASTER_URI'] = 'http://wrover-nano.local:11311'
        os.environ['ROS_IP'] = local_inet_addr

    # run roslaunch
    os.environ['WROVER_HW'] = 'MOCK' if mock_mode else 'REAL'
    os.execlp('roslaunch', 'roslaunch', launch_file, "--screen") # maybe a little unclean, since we aren't cleaning up qt

class LauncherUI(Plugin):
    def __init__(self, context):
        super(LauncherUI, self).__init__(context)
        self.setObjectName('LauncherUI')

        pkg_dir = rospkg.RosPack().get_path('wr_entry_point')

        widget = QWidget()
        ui_file_path = os.path.join(pkg_dir, 'resource', 'launcher_ui.ui')
        pyqt.loadUi(ui_file_path, widget)
        widget.setObjectName('LauncherUIWidget')
        context.add_widget(widget)
        
        w_launch_type = widget.findChild(QComboBox, 'launchType')
        w_local_mode = widget.findChild(QCheckBox, 'localMode')
        w_mock_mode = widget.findChild(QCheckBox, 'mockMode')
        w_launch_btn = widget.findChild(QPushButton, 'launchBtn')

        launch_dir = os.path.join(pkg_dir, 'launch')
        for fname in sorted(os.listdir(launch_dir)):
            if fname.endswith('.launch') and fname != 'launcher_ui.launch':
                w_launch_type.addItem(f'Full: {fname[:-7]}', os.path.join(launch_dir, fname))
        test_launch_dir = os.path.join(launch_dir, 'test')
        for fname in sorted(os.listdir(test_launch_dir)):
            if fname.startswith('test_') and fname.endswith('.launch'):
                w_launch_type.addItem(f'Test: {fname[5:-7]}', os.path.join(test_launch_dir, fname))

        def launch_btn_callback():
            do_launch(w_launch_type.currentData(), w_local_mode.isChecked(), w_mock_mode.isChecked())
            
        w_launch_btn.clicked.connect(launch_btn_callback)
