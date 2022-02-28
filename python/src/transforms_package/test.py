from PyQt5 import QtWidgets
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
from quaternion import *

class Widget(QtWidgets.QDialog):
    def __init__(self, data):
        super().__init__()
        fig = Figure()
        self.fig = fig
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(111, projection='3d', azim=170, elev=20)
        plot_quaternions(self.axes , [data[0]], [data[1]])
        
        mainLayout = QtWidgets.QGridLayout()
        self.topLayout = QtWidgets.QGroupBox()
        self.createTopLayout()

        bottomLayout = QtWidgets.QVBoxLayout()
        mainLayout.addLayout(bottomLayout, 0,0,1,1)

        mainLayout.addWidget(self.topLayout,0, 0)
        mainLayout.addWidget(self.canvas,1,0)

        self.setLayout(mainLayout)
        self.setWindowTitle('3D - plot')

    def createTopLayout(self):
        layout = QtWidgets.QGridLayout()
        label_width = 15

        label_x = QtWidgets.QLabel("x:")
        label_x.setFixedWidth(label_width)

        label_y = QtWidgets.QLabel("y:")
        label_y.setFixedWidth(label_width)

        label_z = QtWidgets.QLabel("z:")
        label_z.setFixedWidth(label_width)
        
        label_w = QtWidgets.QLabel("w:")
        label_w.setFixedWidth(label_width)

        spinBox_x = QtWidgets.QDoubleSpinBox(self.topLayout)
        spinBox_x.setValue(50)

        spinBox_y = QtWidgets.QDoubleSpinBox(self.topLayout)
        spinBox_y.setValue(50)

        spinBox_z = QtWidgets.QDoubleSpinBox(self.topLayout)
        spinBox_z.setValue(50)

        spinBox_w = QtWidgets.QDoubleSpinBox(self.topLayout)
        spinBox_w.setValue(50)

        layout.addWidget(label_x, 0, 0)
        layout.addWidget(spinBox_x, 0, 1)

        layout.addWidget(label_y, 0, 2)
        layout.addWidget(spinBox_y, 0, 3)

        layout.addWidget(label_z, 0, 4)
        layout.addWidget(spinBox_z, 0, 5)

        layout.addWidget(label_w, 0, 6)
        layout.addWidget(spinBox_w, 0, 7)

        layout.setHorizontalSpacing(18)
        self.topLayout.setLayout(layout)

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    q1 = Quaternion( [-0.0880986,0.0015378,0.9959589,0.0173845] )
    t1 = [-1.6602861245985938, 0.0,1.732975221562235]

    win = Widget([q1,t1])
    win.show()
    app.exec()
