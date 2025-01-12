from PyQt5 import QtWidgets, uic
from PyQt5.QtSerialPort import QSerialPort, QSerialPortInfo
from PyQt5.QtCore import QIODevice


# Инициализация приложения
app = QtWidgets.QApplication([])
ui = uic.loadUi("design.ui")
ui.setWindowTitle("SerialGUI")

# Инициализация Serial порта
serial = QSerialPort()
serial.setBaudRate(9600)  # Baud rate должен соответствовать настройкам Arduino

# Список доступных портов
portList = []
ports = QSerialPortInfo().availablePorts()
for port in ports:
    portList.append(port.portName())
ui.ComList.addItems(portList)

# Функция для открытия соединения
def open_serial():
    port_name = ui.ComList.currentText()
    if port_name:
        serial.setPortName(port_name)
        if serial.open(QIODevice.ReadWrite):
            ui.statusbar.showMessage(f"Подключено к {port_name}")
        else:
            ui.statusbar.showMessage("Не удалось открыть порт")

# Функция для закрытия соединения
def close_serial():
    if serial.isOpen():
        serial.close()
        ui.statusbar.showMessage("Соединение закрыто")

# Функция для обработки изменения положения слайдера 1-го мотора
def slider1_changed(value):
    if serial.isOpen():
        command = f"M1:{value}\n"  # Формат команды: M1:<значение>
        serial.write(command.encode())

# Функция для обработки изменения положения слайдера 2-го мотора
def slider2_changed(value):
    if serial.isOpen():
        command = f"M2:{value}\n"  # Формат команды: M2:<значение>
        serial.write(command.encode())

# Связывание кнопок интерфейса с функциями
ui.OpenButton.clicked.connect(open_serial)
ui.CloseButton.clicked.connect(close_serial)

# Связывание слайдеров с функциями
ui.PWM_1motor.valueChanged.connect(slider1_changed)
ui.PWM_1motor_2.valueChanged.connect(slider2_changed)

# Запуск приложения
ui.show()
app.exec()