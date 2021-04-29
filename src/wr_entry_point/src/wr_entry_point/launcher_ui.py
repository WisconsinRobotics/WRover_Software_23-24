import os
import rospkg

from qt_gui.plugin import Plugin
import python_qt_binding as pyqt
from python_qt_binding.QtWidgets import QApplication, QCheckBox, QComboBox, QPushButton, QWidget

def do_launch(launch_file: str, local_mode: bool, mock_mode: bool):
    os.environ['WROVER_LOCAL'] = 'true' if local_mode else 'false'
    os.environ['WROVER_HW'] = 'MOCK' if mock_mode else 'REAL'
    os.execlp('roslaunch', 'roslaunch', launch_file) # maybe a little unclean, since we aren't cleaning up qt

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
