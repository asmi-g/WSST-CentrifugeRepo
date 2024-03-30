import serial
import tkinter as tk

# Open serial port (replace 'COMX' with the actual serial port name)
ser = serial.Serial('COM7', baudrate=115200, timeout=1)

def turnOnLED():
    ser.write(b'255\n\r')

def turnOffLED(): 
    ser.write(b'00\n\r')

def customBrightness():
    brightness = entry.get()
    while len(brightness) <= 2:
        brightness = '0' + brightness
    if brightness.isdigit() and len(brightness) == 3:
        print(brightness.encode())
        ser.write(f'{brightness}\n\r'.encode())
    else:
        print("Invalid brightness value")

def readSerialMon():
    response = ser.readline().decode()
    print("Response from STM32:", response)

# creating tkinter window 
root = tk.Tk() 
root.title('Blink GUI')

btn_On = tk.Button(root, text="Turn On", command=turnOnLED)
btn_On.grid(row=0, column=0)

btn_Off = tk.Button(root, text="Turn Off", command=turnOffLED)
btn_Off.grid(row=0, column=1)

btn_Read = tk.Button(root, text="Read", command=readSerialMon)
btn_Read.grid(row=0, column=2)

# Entry widget for custom input
entry = tk.Entry(root)
entry.grid(row=1, column=0)

# Button to set brightness
btn_SetBrightness = tk.Button(root, text="Set Brightness", command=customBrightness)
btn_SetBrightness.grid(row=1, column=1)

root.geometry("350x350")
root.mainloop()