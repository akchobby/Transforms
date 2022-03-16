from PyQt5 import QtWidgets,QtCore
from transforms_package.gui.toplayout import TopLayout

class Widget(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()


        mainLayout = QtWidgets.QGridLayout()

        # bottom layout
        self.bottomLayout = QtWidgets.QGroupBox()
        self.createBottomLayout()
        
        # top layout        
        self.topLayout = TopLayout(self.txt)
        self.topLayout.create()


        mainLayout.addWidget(self.topLayout, 0, 0)
        mainLayout.addWidget(self.topLayout.canvas, 1,0)
        mainLayout.addWidget(self.bottomLayout, 2,0)

        self.setLayout(mainLayout)
        self.setWindowTitle('Rotation Viewer')
    
    def createBottomLayout(self):
        bLayout = QtWidgets.QVBoxLayout()

        self.txt = QtWidgets.QTextBrowser(self.bottomLayout)
        self.txt.setGeometry(QtCore.QRect())
        self.txt.setObjectName("Params")
        bLayout.addWidget(self.txt, 0)
        self.bottomLayout.setLayout(bLayout)