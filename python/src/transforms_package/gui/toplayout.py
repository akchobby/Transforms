from PyQt5 import QtWidgets, QtGui
import matplotlib
matplotlib.use('Qt5Agg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from transforms_package.quaternion import *


class TopLayout(QtWidgets.QGroupBox):

    def __init__(self, textBox):

        super().__init__()

        self.txt = textBox

        layout =  QtWidgets.QVBoxLayout()
        labels = self._createLabels()
        layout.addWidget(labels[0],0)
        layout.addLayout(self._createRotationLayout(),1)
        layout.addWidget(labels[1],2)
        layout.addLayout(self._createQuaternionLayout(),3)
        layout.addWidget(labels[2],4)
        layout.addLayout(self._createEulerLayout(),5)
        self.setLayout(layout)

        for box in self.quat_spinboxes:
            box.valueChanged.connect(self.quat_update)

        for row in self.rotation_spinboxes:
            for box in row:
                box.valueChanged.connect(self.rotation_update)
        
        for unit in self.units:
            unit.toggled.connect(self.unit_update)

        for box in self.euler_spinboxes:
            box.valueChanged.connect(self.euler_update)

        self.order_box.currentIndexChanged.connect(self._order_update)
        
        # Mid layout declaring here for ease of access
        fig = Figure()
        self.fig = fig
        self.canvas = FigureCanvas(self.fig)
        self.axes = self.fig.add_subplot(111, projection='3d', azim=170, elev=20)
        
    

    def _createRotationLayout(self):
        # Rotation
        rotation_layout = QtWidgets.QGridLayout()
        self.rotation_spinboxes = [[ QtWidgets.QDoubleSpinBox(self) for i in range(0,3)] for i in range(0,3)]

        for i, row in enumerate(self.rotation_spinboxes):   
            for j, box in enumerate(row):  
                box.setMinimum(-1.175494e+38) 
                box.setMaximum(1.175494e+38) 
                box.setDecimals(15) 
                rotation_layout.addWidget(box, i, j)
        
        return rotation_layout


    def _createQuaternionLayout(self):
        # Quaternion
        quat_layout = QtWidgets.QGridLayout()
        label_width = 15
        labels = ["x:","y:","z:","w:"]
        self.quat_spinboxes = [QtWidgets.QDoubleSpinBox(self) for i in labels]
        q_labels = [QtWidgets.QLabel(label) for label in labels]
        j = 0


        for i, box in enumerate(self.quat_spinboxes):   

            # fix width for UI
            q_labels[i].setFixedWidth(label_width)

            quat_layout.addWidget(q_labels[i], 0 , j)
            j += 1
            box.setMinimum(-1.175494e+38) 
            box.setMaximum(1.175494e+38) 
            box.setDecimals(15) 
            quat_layout.addWidget(box, 0, j)
            j += 1


        quat_layout.setHorizontalSpacing(18)

        return quat_layout

    def _createEulerLayout(self):
        # euler
        euler_layout = QtWidgets.QGridLayout()
        labels = ["x:","y:","z:","w:"]
        label_width = 15
        self.order_list = ["zyx","xyz"]
        self.order = self.order_list [0]

        self.order_box = QtWidgets.QComboBox()
        self.order_box.addItems(self.order_list)
        euler_layout.addWidget(self.order_box, 0 , 0)


        self.deg =True
        self.units = [QtWidgets.QRadioButton("deg"), QtWidgets.QRadioButton("radians")]
        self.units[0].setChecked(self.deg)

        self.units[0].unit = "deg"
        self.units[1].unit = "radians"

        for i,unit_box in enumerate(self.units):
            euler_layout.addWidget(unit_box, 1 , i)

        self.euler_spinboxes = [QtWidgets.QDoubleSpinBox(self) for i in labels[:-1]]
        euler_labels = [QtWidgets.QLabel(label) for label in labels[:-1]]
        j = 0

        for i, box in enumerate(self.euler_spinboxes):   

            # fix width for UI
            euler_labels[i].setFixedWidth(label_width)

            euler_layout.addWidget(euler_labels[i], 2 , j)
            j += 1
            box.setMinimum(-1.175494e+38) 
            box.setMaximum(1.175494e+38) 
            box.setDecimals(15) 
            euler_layout.addWidget(box, 2, j)
            j += 1


        euler_layout.setHorizontalSpacing(18)  

        return euler_layout

    def _createLabels(self):
        rotation_label = QtWidgets.QLabel("Rotation Matrix :")
        quat_label = QtWidgets.QLabel("Quaternion :")
        euler_label = QtWidgets.QLabel("Euler intrinsic:")
        
        bold_font = QtGui.QFont()
        bold_font.setBold(True)
        rotation_label.setFont(bold_font)
        quat_label.setFont(bold_font)
        euler_label.setFont(bold_font)

        return rotation_label,quat_label, euler_label
    
    def quat_update(self):
        try:
            print("[INFO] quaternion update")
            q = Quaternion([box.value() for box in self.quat_spinboxes])

            # update euler
            self._silent_update_euler(q)

            # update rotation mat
            self._silent_update_rotation(q)

            
            # update text box
            self._update_text_display(q)

            self.axes.cla()
            plot_quaternions(self.axes, q)
            self.canvas.draw()

        except Exception as e:
            print(e)
            self.axes.cla()

    
    def rotation_update(self):
        try:
            print("[INFO] rotation update")
            q = Quaternion([[box.value() for box in row ]for row in self.rotation_spinboxes])

            # update euler
            self._silent_update_euler(q)

            # update quaternion
            self._silent_update_quaternion(q)

            # update text box
            self._update_text_display(q)

            self.axes.cla()
            plot_quaternions(self.axes, q)
            self.canvas.draw()
        except:
            self.axes.cla()
    
    def euler_update(self):
        try:
            print("[INFO] euler angles update")
            q = Quaternion([box.value() for box in self.euler_spinboxes], deg=self.deg, order=self.order)

            # update quaternion
            self._silent_update_quaternion(q)

            # update rotation mat
            self._silent_update_rotation(q)
            
            # update text box
            self._update_text_display(q)

            self.axes.cla()
            plot_quaternions(self.axes, q)
            self.canvas.draw()
        except Exception as e:
            print(e)
            self.axes.cla()
    
    def _silent_update_euler(self,q):
        # update euler
        for i, val in enumerate(q.to_euler(deg=self.deg, order=self.order)):
            self.euler_spinboxes[i].blockSignals(True)
            self.euler_spinboxes[i].setValue(val)
            self.euler_spinboxes[i].blockSignals(False)

    def _silent_update_quaternion(self,q):
        # update quaternion
        for i, val in enumerate(q.to_list()):
            self.quat_spinboxes[i].blockSignals(True)
            self.quat_spinboxes[i].setValue(val)
            self.quat_spinboxes[i].blockSignals(False)

    def _silent_update_rotation(self,q):
        # update rotation mat
        rotation_matrix = q.to_rotation_matrix()
        for i,row in enumerate(rotation_matrix):
            for j, val in enumerate(row):
                self.rotation_spinboxes[i][j].blockSignals(True)
                self.rotation_spinboxes[i][j].setValue(val)
                self.rotation_spinboxes[i][j].blockSignals(False)
    
    def _update_text_display(self,q):
        self.txt.clear()
        self.txt.textCursor().insertText(" ".join(["Unit Quaternion:\t", str(q.to_list()), "\nEuler",self.order,":\t\t", str(q.to_euler(deg=self.deg, order=self.order))]))
    
    def _order_update(self,i):
        self.order = self.order_list[i]
        self.euler_update()
        #self._silent_update_euler(Quaternion([box.value() for box in self.euler_spinboxes], deg=self.deg, order=self.order))





    
    def unit_update(self):
        button = self.sender()
        self.deg = (button.unit == "deg" and button.isChecked() )
    

    

