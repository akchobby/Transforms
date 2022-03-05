from PyQt5 import QtWidgets, QtGui
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

        mainLayout = QtWidgets.QGridLayout()
        self.topLayout = QtWidgets.QGroupBox()
        self.createTopLayout()
        for box in self.quat_spinboxes:
            box.valueChanged.connect(self.quat_update)

        for row in self.rotation_spinboxes:
            for box in row:
                box.valueChanged.connect(self.rotation_update)

        bottomLayout = QtWidgets.QVBoxLayout()
        mainLayout.addLayout(bottomLayout, 0,0,1,1)

        mainLayout.addWidget(self.topLayout,0, 0)
        mainLayout.addWidget(self.canvas,1,0)

        self.setLayout(mainLayout)
        self.setWindowTitle('Rotation Viewer')

    def createTopLayout(self):
        layout =  QtWidgets.QVBoxLayout()
        # Rotation
        rotation_layout = QtWidgets.QGridLayout()
        self.rotation_spinboxes = [[ QtWidgets.QDoubleSpinBox(self.topLayout) for i in range(0,3)] for i in range(0,3)]

        for i, row in enumerate(self.rotation_spinboxes):   
            for j, box in enumerate(row):  
                box.setMinimum(-1.175494e+38) 
                rotation_layout.addWidget(box, i, j)

        # Quaternion
        quat_layout = QtWidgets.QGridLayout()
        label_width = 15
        labels = ["x:","y:","z:","w:"]
        self.quat_spinboxes = [QtWidgets.QDoubleSpinBox(self.topLayout) for i in labels]
        q_labels = [QtWidgets.QLabel(label) for label in labels]
        j = 0


        for i, box in enumerate(self.quat_spinboxes):   

            # fix width for UI
            q_labels[i].setFixedWidth(label_width)

            quat_layout.addWidget(q_labels[i], 0 , j)
            j += 1
            box.setMinimum(-1.175494e+38) 
            quat_layout.addWidget(box, 0, j)
            j += 1


        quat_layout.setHorizontalSpacing(18)

        rotation_label = QtWidgets.QLabel("Rotation Matrix :")
        quat_label = QtWidgets.QLabel("Quaternion :")
        
        bold_font = QtGui.QFont()
        bold_font.setBold(True)
        rotation_label.setFont(bold_font)
        quat_label.setFont(bold_font)
        layout.addWidget(rotation_label,0)
        layout.addLayout(rotation_layout,1)
        layout.addWidget(quat_label,2)
        layout.addLayout(quat_layout,3)
        self.topLayout.setLayout(layout)

    def quat_update(self):
        try:
            print("quaternion update")
            q = Quaternion([box.value() for box in self.quat_spinboxes])
            rotation_matrix = q.to_rotation_matrix()


            for i,row in enumerate(rotation_matrix):
                for j, val in enumerate(row):
                    self.rotation_spinboxes[i][j].blockSignals(True)
                    self.rotation_spinboxes[i][j].setValue(val)
                    self.rotation_spinboxes[i][j].blockSignals(False)

            self.axes.cla()
            plot_quaternions(self.axes, q)
            self.canvas.draw()

        except Exception as e:
            print(e)
            self.axes.cla()

    
    def rotation_update(self):
        try:
            print("rotation update")
            q = Quaternion([[box.value() for box in row ]for row in self.rotation_spinboxes])

            
            for i, val in enumerate(q.to_list()):
                self.quat_spinboxes[i].blockSignals(True)
                self.quat_spinboxes[i].setValue(val)
                self.quat_spinboxes[i].blockSignals(False)

            self.axes.cla()
            plot_quaternions(self.axes, q)
            self.canvas.draw()
        except:
            self.axes.cla()




if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    q1 = Quaternion( [0.5, 0.5,0.5,0.5] )
    print(q1.to_rotation_matrix())
    t1 = [-1.6602861245985938, 0.0,1.732975221562235]

    win = Widget([q1,t1])
    win.show()
    app.exec()
