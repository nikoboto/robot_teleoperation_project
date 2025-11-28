#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import tkinter as tk
from tkinter import ttk

class GuiCommander:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Control GUI")
        
        self.pub = rospy.Publisher('/comandos', Float64MultiArray, queue_size=10)
        rospy.init_node('gui_commander', anonymous=True)
        
        # Source ID for GUI (e.g., 3)
        self.source_id = 3
        
        self.create_widgets()
        
    def create_widgets(self):
        # Frame for Inputs
        input_frame = ttk.LabelFrame(self.root, text="Parameters")
        input_frame.grid(row=0, column=0, padx=10, pady=10, sticky="ew")
        
        ttk.Label(input_frame, text="Var 1 (X / Roll / LinX):").grid(row=0, column=0, padx=5, pady=5)
        self.var1_entry = ttk.Entry(input_frame)
        self.var1_entry.insert(0, "0.0")
        self.var1_entry.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Label(input_frame, text="Var 2 (Y / Pitch / LinY):").grid(row=1, column=0, padx=5, pady=5)
        self.var2_entry = ttk.Entry(input_frame)
        self.var2_entry.insert(0, "0.0")
        self.var2_entry.grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Label(input_frame, text="Var 3 (Z / Yaw / LinZ):").grid(row=2, column=0, padx=5, pady=5)
        self.var3_entry = ttk.Entry(input_frame)
        self.var3_entry.insert(0, "0.0")
        self.var3_entry.grid(row=2, column=1, padx=5, pady=5)
        
        ttk.Label(input_frame, text="Time (s):").grid(row=3, column=0, padx=5, pady=5)
        self.time_entry = ttk.Entry(input_frame)
        self.time_entry.insert(0, "1.0")
        self.time_entry.grid(row=3, column=1, padx=5, pady=5)
        
        # Frame for Buttons
        btn_frame = ttk.LabelFrame(self.root, text="Commands")
        btn_frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")
        
        ttk.Button(btn_frame, text="Pos Position (1)", command=lambda: self.send_command(1)).grid(row=0, column=0, padx=5, pady=5)
        ttk.Button(btn_frame, text="Pos Orientation (2)", command=lambda: self.send_command(2)).grid(row=0, column=1, padx=5, pady=5)
        ttk.Button(btn_frame, text="Vel Linear (3)", command=lambda: self.send_command(3)).grid(row=1, column=0, padx=5, pady=5)
        ttk.Button(btn_frame, text="Vel Angular (4)", command=lambda: self.send_command(4)).grid(row=1, column=1, padx=5, pady=5)
        
        ttk.Button(btn_frame, text="STOP", command=self.stop_robot).grid(row=2, column=0, columnspan=2, padx=5, pady=10, sticky="ew")

    def get_values(self):
        try:
            v1 = float(self.var1_entry.get())
            v2 = float(self.var2_entry.get())
            v3 = float(self.var3_entry.get())
            t = float(self.time_entry.get())
            return v1, v2, v3, t
        except ValueError:
            print("Invalid input")
            return 0.0, 0.0, 0.0, 0.0

    def send_command(self, option):
        v1, v2, v3, t = self.get_values()
        msg = Float64MultiArray()
        # [option, var1, var2, var3, time, source_id]
        msg.data = [float(option), v1, v2, v3, t, float(self.source_id)]
        self.pub.publish(msg)
        print(f"Sent command: Opt={option}, V=({v1},{v2},{v3}), T={t}, ID={self.source_id}")

    def stop_robot(self):
        # Send zero velocity command
        msg = Float64MultiArray()
        # Option 3 (Linear Vel) with 0 values is a safe stop
        msg.data = [3.0, 0.0, 0.0, 0.0, 1.0, float(self.source_id)]
        self.pub.publish(msg)
        print("STOP command sent")

if __name__ == "__main__":
    root = tk.Tk()
    app = GuiCommander(root)
    root.mainloop()
