import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtCore import QStringListModel, QModelIndex
from PyQt5 import uic

import roboszpon_lib


class MyLittleRoboszponSuite(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("app.ui", self)
        self.roboszpon = 0
        self.armed = False

        self.deviceList = QStringListModel()
        self.devices = []
        self.deviceListView.setModel(self.deviceList)
        self.deviceListView.clicked.connect(self.deviceListClicked)

        self.armButton.clicked.connect(self.armButtonClicked)
        self.dutyButton.clicked.connect(self.dutyButtonClicked)
        self.velocityButton.clicked.connect(self.velocityButtonClicked)
        self.positionButton.clicked.connect(self.positionButtonClicked)
        self.stopAllButton.clicked.connect(self.stopAllButtonClicked)

        self.addConnectedDevice(1)
        self.addConnectedDevice(2)
        roboszpon_lib.send_can_frame("can0", 0x01, 0x003F000000000000)

        # Connect the submit button to the submit_parameters method

    #     self.submit_button.clicked.connect(self.submit_parameters)
    def addConnectedDevice(self, node_id):
        self.deviceList.insertRow(self.deviceList.rowCount())
        index = self.deviceList.index(self.deviceList.rowCount() - 1)
        self.deviceList.setData(index, f"Roboszpon 0x{node_id:02x}", 0)
        self.devices.append(node_id)
        print(self.devices)

    def removeConnectedDevice(self, node_id):
        if node_id not in self.devices:
            raise KeyError("can't remove node. there is no such node")
        index = self.devices.index(node_id)
        self.deviceList.removeRow(index)
        self.devices.remove(node_id)
        print(self.devices)

    def selectDevice(self, node_id):
        self.roboszpon = node_id
        # set self.armed to armed or disarmed accordingly
        print(f"Selected roboszpon: {node_id}")

    def deviceListClicked(self, index: QModelIndex):
        self.selectDevice(self.devices[index.row()])

    def armButtonClicked(self):
        self.armed = not self.armed
        if self.armed:
            roboszpon_lib.disarm("can0", self.roboszpon)
            self.armButton.setText("Arm")
        else:
            roboszpon_lib.arm("can0", self.roboszpon)
            self.armButton.setText("Disarm")

    def stopAllButtonClicked(self):
        roboszpon_lib.emergency_stop("can0")
        self.armed = False
        self.armButton.setText("Disarm")

    def dutyButtonClicked(self):
        roboszpon_lib.send_duty_command(
            "can0", self.roboszpon, self.dutySpinBox.value()
        )

    def velocityButtonClicked(self):
        roboszpon_lib.send_velocity_command(
            "can0", self.roboszpon, self.velocitySpinBox.value()
        )

    def positionButtonClicked(self):
        roboszpon_lib.send_position_command(
            "can0", self.roboszpon, self.positionSpinBox.value()
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyLittleRoboszponSuite()
    ex.show()
    sys.exit(app.exec_())
