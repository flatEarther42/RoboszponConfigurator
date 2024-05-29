import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox
from PyQt5.QtCore import QStringListModel, QModelIndex, QTimer
from PyQt5 import uic

import roboszpon_lib
import can, time

CONNECTION_TIMEOUT = 1.0


class Roboszpon:
    node_id: int
    mode: str
    flags: int
    timestamp: float


class MyLittleRoboszponSuite(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("app.ui", self)

        self.canbus = can.interface.Bus("can0", interface="socketcan")

        self.roboszpon = None
        self.armed = False

        self.deviceList = QStringListModel()
        self.deviceIds = []
        self.devices = {}
        self.deviceListView.setModel(self.deviceList)
        self.deviceListView.clicked.connect(self.deviceListClicked)

        self.armButton.clicked.connect(self.armButtonClicked)
        self.dutyButton.clicked.connect(self.dutyButtonClicked)
        self.velocityButton.clicked.connect(self.velocityButtonClicked)
        self.positionButton.clicked.connect(self.positionButtonClicked)
        self.stopAllButton.clicked.connect(self.stopAllButtonClicked)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        can.Notifier(self.canbus, [self])

    def __del__(self):
        self.canbus.shutdown()

    def __call__(self, message):
        self.on_message_received(message)

    def tick(self):
        if self.roboszpon is not None:
            if (
                self.devices[self.roboszpon].timestamp + CONNECTION_TIMEOUT
                < time.time()
            ):
                self.connectionLabel.setText(" Connection: LOST")
            self.stateLabel.setText(
                f" Operation state: {roboszpon_lib.ROBOSZPON_MODES[self.devices[self.roboszpon].mode]}"
            )
            self.flagsLabel.setText(
                f" Flags: {(bin(self.devices[self.roboszpon].flags + (1 << 33)))[3:]}"
            )

    def on_message_received(self, message: can.message.Message):
        if message.is_extended_id or message.dlc != 8:
            print("Received invalid CAN message")
        msg = roboszpon_lib.decode_message(
            message.arbitration_id, int.from_bytes(message.data, "big")
        )
        if msg["message_id"] == roboszpon_lib.MSG_STATUS_REPORT:
            node_id = msg["node_id"]
            if node_id not in self.deviceIds:
                self.addConnectedDevice(node_id)
            roboszpon = self.devices[msg["node_id"]]
            roboszpon.mode = msg["mode"]
            roboszpon.flags = msg["flags"]
            roboszpon.timestamp = message.timestamp
            if msg["node_id"] == self.roboszpon:
                self.connectionLabel.setText(" Connection: OK")

    def addConnectedDevice(self, node_id):
        self.deviceList.insertRow(self.deviceList.rowCount())
        index = self.deviceList.index(self.deviceList.rowCount() - 1)
        self.deviceList.setData(index, f"Roboszpon 0x{node_id:02x}", 0)
        self.deviceIds.append(node_id)
        self.devices[node_id] = Roboszpon()
        self.devices[node_id].node_id = node_id
        print(self.deviceIds)

    def removeConnectedDevice(self, node_id):
        if node_id not in self.deviceIds:
            raise KeyError("can't remove node. there is no such node")
        index = self.deviceIds.index(node_id)
        self.deviceList.removeRow(index)
        self.deviceIds.remove(node_id)
        print(self.deviceIds)

    def selectDevice(self, node_id):
        self.roboszpon = node_id
        self.armed = self.devices[node_id].mode != roboszpon_lib.ROBOSZPON_MODE_STOPPED
        if self.armed:
            self.armButton.setText("Disarm")
        else:
            self.armButton.setText("Arm")
        print(f"Selected roboszpon: {node_id}")

    def deviceListClicked(self, index: QModelIndex):
        self.selectDevice(self.deviceIds[index.row()])

    def armButtonClicked(self):
        self.armed = not self.armed
        if self.armed:
            roboszpon_lib.disarm(self.canbus, self.roboszpon)
            self.armButton.setText("Arm")
        else:
            roboszpon_lib.arm(self.canbus, self.roboszpon)
            self.armButton.setText("Disarm")

    def stopAllButtonClicked(self):
        roboszpon_lib.emergency_stop(self.canbus)
        self.armed = False
        self.armButton.setText("Disarm")

    def dutyButtonClicked(self):
        roboszpon_lib.send_duty_command(
            self.canbus, self.roboszpon, self.dutySpinBox.value()
        )

    def velocityButtonClicked(self):
        roboszpon_lib.send_velocity_command(
            self.canbus, self.roboszpon, self.velocitySpinBox.value()
        )

    def positionButtonClicked(self):
        roboszpon_lib.send_position_command(
            self.canbus, self.roboszpon, self.positionSpinBox.value()
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyLittleRoboszponSuite()
    ex.show()
    sys.exit(app.exec_())
