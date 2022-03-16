from transforms_package.gui.mainWindow import Widget
from PyQt5 import QtWidgets

if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    win = Widget()
    win.show()
    app.exec()
