import tkinter as tk
import serial
import threading
import queue

class UARTReader:
    def __init__(self, port, baudrate):
        self.serial_port = serial.Serial(port, baudrate, timeout=1)
        self.reading = True
        self.queue = queue.Queue()

    def read_from_uart(self):
        print("Starting UART read thread...")
        while self.reading:
            if self.serial_port.in_waiting > 0:
                data = self.serial_port.readline().decode('utf-8').strip()
                print(f"Received data: {data}")
                self.queue.put(data)
        print("Stopped UART read thread...")

    def send_to_uart(self, data):
        self.serial_port.write(data.encode('utf-8'))
        print(f"Sent command: {data}")

    def stop_reading(self):
        self.reading = False
        self.serial_port.close()
        print("Serial port closed.")

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("UART Reader")

        self.value_labels = []
        for i in range(10):
            row = i % 2
            col = (i // 2) % 5
            label = tk.Label(root, text=f"Temp {i+1}:")
            label.grid(row=row*2, column=col, padx=10, pady=5)
            value = tk.Label(root, text="0")
            value.grid(row=row*2+1, column=col, padx=10, pady=5)
            self.value_labels.append(value)
            
        self.signal_lights = []
        for i in range(4):
            row = 4
            col = i
            canvas = tk.Canvas(root, width=18, height=18)
            canvas.grid(row=row, column=col, padx=10, pady=5)
            light = canvas.create_oval(2, 2, 18, 18, fill="gray")
            self.signal_lights.append((canvas, light))

        self.start_button = tk.Button(root, text="Start Reading", command=self.start_reading)
        self.start_button.grid(row=7, column=0, pady=10)

        self.stop_button = tk.Button(root, text="Stop Reading", command=self.stop_reading)
        self.stop_button.grid(row=7, column=1, pady=10)

        self.uart_reader = UARTReader('COM8', 115200)  # Adjust the COM port and baud rate as needed

        self.create_control_buttons()

    def create_control_buttons(self):
        button_texts = [
            "Pre-Heat 0", "Pre-Heat 1", "Pre-Heat 2", "Pre-Heat 3",
            "Full Heat 0", "Full Heat 1", "Full Heat 2", "Full Heat 3",
            "Motor On", "Motor Off"
        ]
        button_commands = [
            self.send_command_1, self.send_command_2, self.send_command_3, self.send_command_4,
            self.send_command_5, self.send_command_6, self.send_command_7, self.send_command_8,
            self.motor_on, self.motor_off
        ]
        for i, (text, command) in enumerate(zip(button_texts, button_commands)):
            button = tk.Button(self.root, text=text, command=command)
            row = 5 + i // 4
            col = i % 4
            button.grid(row=row, column=col, padx=10, pady=10)

    def send_command_1(self):
        self.uart_reader.send_to_uart("CMD1\n")

    def send_command_2(self):
        self.uart_reader.send_to_uart("CMD2\n")

    def send_command_3(self):
        self.uart_reader.send_to_uart("CMD3\n")

    def send_command_4(self):
        self.uart_reader.send_to_uart("CMD4\n")

    def send_command_5(self):
        self.uart_reader.send_to_uart("CMD5\n")

    def send_command_6(self):
        self.uart_reader.send_to_uart("CMD6\n")

    def send_command_7(self):
        self.uart_reader.send_to_uart("CMD7\n")

    def send_command_8(self):
        self.uart_reader.send_to_uart("CMD8\n")

    def motor_on(self):
        self.uart_reader.send_to_uart("CMD9\n")

    def motor_off(self):
        self.uart_reader.send_to_uart("CMD10\n")
        
    def start_reading(self):
        print("Starting to read UART data...")
        self.uart_reader.reading = True
        self.reading_thread = threading.Thread(target=self.uart_reader.read_from_uart)
        self.reading_thread.start()
        self.root.after(100, self.process_serial_data)

    def stop_reading(self):
        print("Stopping UART reading...")
        self.uart_reader.stop_reading()
        self.reading_thread.join()

    def process_serial_data(self):
        while not self.uart_reader.queue.empty():
            data = self.uart_reader.queue.get()
            print(f"Processing data: {data}")
            self.display_data(data)
        self.root.after(100, self.process_serial_data)
    
    def update_signal_light(self, index, value):
        canvas, light = self.signal_lights[index]
        if int(value) == 0:
            color = "gray"
        elif int(value) == 1:
            color = "yellow"
        elif int(value) == 2:
            color = "red"
        else:
            color = "gray"  # Default to gray for any unexpected values
        canvas.itemconfig(light, fill=color)

    def display_data(self, data):
        values = data.split(',')
        if len(values) == 14:
            for i in range(10):
                self.value_labels[i].config(text=values[i])
            for i in range(4):
                self.update_signal_light(i, values[i + 10])
        else:
            print("Received data does not contain 10 values.")     

if __name__ == "__main__":
    root = tk.Tk()
    app = App(root)
    root.mainloop()