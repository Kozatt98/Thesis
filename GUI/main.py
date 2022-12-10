import sys

from PyQt5.QtWidgets import *
from main_window import Window


def main():
    app = QApplication(sys.argv)
    window = Window()
    app.aboutToQuit.connect(window.cleanup)
    window.showMaximized()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
