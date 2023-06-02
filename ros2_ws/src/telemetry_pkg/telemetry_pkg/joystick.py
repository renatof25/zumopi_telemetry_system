from PyQt5.QtCore import Qt, QPoint, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QBrush
from PyQt5.QtWidgets import QWidget

class Joystick(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        
        # Set widget size
        self.setFixedSize(150, 150)
        
        # Joystick position
        self.x = self.width() // 2
        self.y = self.height() // 2
        
        # Joystick value
        self.value_x = 0
        self.value_y = 0
        
        # Timer to update joystick position
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(10)
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        # Draw joystick background
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(QBrush(QColor(192, 192, 192)))
        painter.drawEllipse(10, 10, self.width() - 20, self.height() - 20)
        
        # Draw joystick handle
        painter.setPen(QPen(Qt.black, 1))
        painter.setBrush(QBrush(QColor(0, 0, 255)))
        painter.drawEllipse(QPoint(self.x, self.y), 15, 15)
        
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.setHandlePosition(event.pos())
            
    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.setHandlePosition(event.pos())
            
    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.setHandlePosition(QPoint(self.width() // 2, self.height() // 2))
            
    def setHandlePosition(self, pos):
        # Calculate joystick position
        dx = pos.x() - self.width() / 2
        dy = pos.y() - self.height() / 2
        dist = (dx ** 2 + dy ** 2) ** 0.5
        if dist > (self.width() - 40) / 2:
            dx = dx * (self.width() - 40) / (2 * dist)
            dy = dy * (self.height() - 40) / (2 * dist)
        self.x = int(self.width() / 2 + dx)
        self.y = int(self.height() / 2 + dy)
        
        # Calculate joystick value (-1 to 1)
        self.value_x = (self.x - (self.width() / 2)) / ((self.width() - 40) / 2)
        self.value_y = ((self.height() / 2) - self.y) / ((self.height() - 40) / 2)

    def getValue(self):
        return self.value_x, self.value_y
