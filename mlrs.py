import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QMessageBox, QVBoxLayout
from PyQt5.QtCore import QStringListModel, QModelIndex, QTimer
from PyQt5.QtGui import QPixmap
from PyQt5 import uic

import roboszpon_lib
import can, time
import pyqtgraph as pg

CONNECTION_TIMEOUT = 1.0
MAX_PLOT_SAMPLES = 2137


class Signal:
    def __init__(self):
        self.values = []
        self.timestamps = []

    def update(self, value, timestamp):
        if len(self.values) == MAX_PLOT_SAMPLES:
            self.values.pop(0)
        self.values.append(value)
        if len(self.timestamps) == MAX_PLOT_SAMPLES:
            self.timestamps.pop(0)
        self.timestamps.append(timestamp)


class Roboszpon:
    def __init__(self):
        self.duty = Signal()
        self.current = Signal()
        self.velocity = Signal()
        self.position = Signal()

    node_id: int
    mode: str
    flags: int
    timestamp: float


class MyLittleRoboszponSuite(QMainWindow):
    def __init__(self):
        super().__init__()
        uic.loadUi("app.ui", self)
        self.logoLabel.setPixmap(QPixmap("assets/text1.png"))
        self.startTimestamp = time.time()

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

        self.init_plot()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)

        try:
            self.canbus = can.interface.Bus("can0", interface="socketcan")
            can.Notifier(self.canbus, [self])
        except Exception as e:
            print(f"Couldn't start can: {e}")

    def __del__(self):
        self.canbus.shutdown()

    def __call__(self, message):
        self.on_message_received(message)

    def tick(self):
        if self.roboszpon is not None:
            self.updatePlot()
            if (
                self.devices[self.roboszpon].timestamp + CONNECTION_TIMEOUT
                < time.time()
            ):
                self.connectionLabel.setText("Connection: LOST")
                self.stateLabel.setText("")
                self.flagsLabel.setText("")

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
                self.connectionLabel.setText("Connection: OK")
                self.stateLabel.setText(
                    f"Operation state: {roboszpon_lib.ROBOSZPON_MODES[self.devices[self.roboszpon].mode]}"
                )
                self.flagsLabel.setText(
                    f"Flags: {(bin(self.devices[self.roboszpon].flags + (1 << 17)))[3:]}"
                )
        if msg["message_id"] == roboszpon_lib.MSG_AXIS_REPORT:
            node_id = msg["node_id"]
            if node_id in self.deviceIds:
                self.devices[node_id].position.update(
                    msg["position"], message.timestamp - self.startTimestamp
                )
                self.devices[node_id].velocity.update(
                    msg["velocity"], message.timestamp - self.startTimestamp
                )
        if msg["message_id"] == roboszpon_lib.MSG_MOTOR_REPORT:
            node_id = msg["node_id"]
            if node_id in self.deviceIds:
                self.devices[node_id].current.update(
                    msg["current"], message.timestamp - self.startTimestamp
                )
                self.devices[node_id].duty.update(
                    msg["duty"], message.timestamp - self.startTimestamp
                )

    def addConnectedDevice(self, node_id):
        self.deviceList.insertRow(self.deviceList.rowCount())
        index = self.deviceList.index(self.deviceList.rowCount() - 1)
        self.deviceList.setData(index, f"Roboszpon 0x{node_id:02x}", 0)
        self.deviceIds.append(node_id)
        self.devices[node_id] = Roboszpon()
        self.devices[node_id].node_id = node_id

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
        self.connectionLabel.setText("Connection: OK")
        self.stateLabel.setText(
            f"Operation state: {roboszpon_lib.ROBOSZPON_MODES[self.devices[self.roboszpon].mode]}"
        )
        self.flagsLabel.setText(
            f"Flags: {(bin(self.devices[self.roboszpon].flags + (1 << 17)))[3:]}"
        )
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

    def init_plot(self):
        self.plot = pg.PlotWidget()
        self.plot.addLegend(offset=-1)
        layout = QVBoxLayout(self.plotView)
        layout.addWidget(self.plot)
        self.duty_curve = self.plot.plot([], [], pen="#7E2F8E", name="Duty")
        self.current_curve = self.plot.plot([], [], pen="#EDB120", name="Current")
        self.velocity_curve = self.plot.plot([], [], pen="#D95319", name="Velocity")
        self.position_curve = self.plot.plot([], [], pen="#0072BD", name="Position")

    def updatePlot(self):
        signal = self.devices[self.roboszpon].position
        self.position_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].velocity
        self.velocity_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].current
        self.current_curve.setData(signal.timestamps, signal.values)
        signal = self.devices[self.roboszpon].duty
        self.duty_curve.setData(signal.timestamps, signal.values)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    ex = MyLittleRoboszponSuite()
    ex.show()
    sys.exit(app.exec_())
