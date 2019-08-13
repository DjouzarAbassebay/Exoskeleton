"""----------------------------------------------------------------------------
--------------Medical Hand Exoskeleton to assess spastic patient---------------
-------------------------------------------------------------------------------

@author: Djouzar ABASSEBAY (Postgraduate Student) and William McColl (Phd student)
Date of last update : 8th of August 2019
This program is composed of three classes using Tkinter framework for the GUI 
and ROS publisher for the output values (separated codes)

------------------------ Imports and variables ------------------------------"""

# imports
import os
import sys
import serial
import time
from datetime import datetime
import numpy
import math
import numpy as np
from std_msgs.msg import Float64, String

#import for ROS
import rospy
from geometry_msgs.msg import Twist
from dynamixel_msgs.msg import JointState as JointStateDynamixel

# tkinter librairies
import tkinter as tk
from tkinter import *
from tkinter import ttk
from tkinter import messagebox

#Save into a pdf file
import pandas as pd
import matplotlib
from pylab import title, figure, xlabel, ylabel, xticks, bar, legend, axis, savefig
from fpdf import FPDF

# For the graphs
from scipy import signal
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import interactive
from matplotlib import style
style.use('ggplot')
from matplotlib.figure import Figure

# Try/catch to check if the connection is established 
try:
    PORT = '/dev/ttyUSB0'
    SPEED = 9600
    connection = serial.Serial(PORT, SPEED, timeout=1)
except OSError as e:
    messagebox.showwarning("Error", "The connection with the Arduino is not establisehd, change the Serial Port and restart the GUI")

global checkFirstValue  # Used to reset the angles values at 0 when the GUI start 
checkFirstValue = 0
global first_value
first_value = 0

global selected_joint  # The joint that will be assessed

# Top sensor Value and array
global TopValue
TopValue = 0
global top_sensor_array # Array with all the values
top_sensor_array = [0]

# Bottom sensor Value 
global BottomValue 
BottomValue = 0
global bottom_sensor_array
bottom_sensor_array = [0]

# Get the times for the plots
global first_time
first_time = time.time()
global now # Used in the plot of pressure sensor
now = []
global time_assessment_begin  # Used to plot just the time of the assessment 
time_assessment_begin = 0
global time_assessment_end
time_assessment_end = 0
global timejoint   # Used in the plot of the angles
timejoint =0
global TimeJointValues_array
TimeJointValues_array = []

# For the settings window
global MCP_Angle_value
global PIP_Angle_value
global DIP_Angle_value
global V1_MCP_value
global V1_PIP_value
global V1_PIP_value
global V2_MCP_value
global V2_PIP_value
global V2_DIP_value
global V3_MCP_value
global V3_PIP_value
global V3_DIP_value
global top_sensor_threshold
top_sensor_threshold = 150
global bottom_sensor_threshold
bottom_sensor_threshold = 150

# DIP, PIP and MCP angles and array
global MCPjointValue
MCPjointValue =0
global MCPjointValue_array
MCPjointValue_array = []
global Pip_angle
Pip_angle = 0
global Pip_angle_array
Pip_angle_array = []
global Dip_angle
Dip_angle = 0
global Dip_angle_array
Dip_angle_array = []

# Used to saved the patient's info
global FirstName
global LastName
global which_assessment

# Colors used
black_color = '#000000'
white_color = '#ffffff'
blue_color = '#006699'
grey_color = '#d9d9d9'

'''-----------------------------  All Functions classes ---------------------------'''
# Shared functions 
class AllFunction():
# Publish the velocity command into ROS Publisher
    def CommandVel(self,ID,vel):
        global pub
        # Check the ID of the motor
        if ID is 1:
            pub = rospy.Publisher('/Knuckle_controller/command', Float64, queue_size=1)
        elif ID is 2:
            pub = rospy.Publisher('/DIP_controller/command', Float64, queue_size=1)
        elif ID is 3:
            pub = rospy.Publisher('/Flexor_controller/command', Float64, queue_size=1)
        rate = rospy.Rate(5) # 5hz
        Motor_vel = vel
        pub.publish(Motor_vel)
        return True

# Stop the motors
    def stop(self):
        self.CommandVel(1,0.00)
        self.CommandVel(2,0.00)
        self.CommandVel(3,0.00)
        print ("Assessment stopped")


'''---------------------------------- Main page -----------------------------------'''
class MainPage:
# Contructor of the main page
    def __init__(self, master):
        #Create an instance of the AllFunction class
        self.functions = AllFunction()

        #Create the window
        self.master = master
        self.master.geometry ("900x1300+40+10")
        self.master.title("Medical Hand Exoskeleton Controller")
        self.master.configure(background="#dadfe1")

        # Create the Menu bar 
        menu = Menu(self.master)
        self.master.config(menu=menu)
        self.file = Menu(menu)
        self.file.add_command(label="New", font='Helvetica 13', command=self.patient_info)
        self.file.add_command(label="Save", font='Helvetica 13', command=self.SaveInfo)
        self.file.add_command(label="Test", font='Helvetica 13', command = self.technical_window)
        self.file.add_command(label="Settings", font='Helvetica 13', command = self.settings_window)
        self.file.add_command(label="Close", font='Helvetica 13', command=self.close_windows)
        menu.add_cascade(label="File", font='Helvetica 13',  menu=self.file)
        self.Help = Menu(menu)
        self.Help.add_command(label="About", font='Helvetica 13', command = self.about_window)
        menu.add_cascade(label="Help",font='Helvetica 13', menu=self.Help)

   # Top Frame
        self.TopFrame = tk.Frame(master)
        
        self.TopFrame.place(relx=0.008, rely=0.01, relheight=0.13, relwidth=0.977)
        self.TopFrame.configure(relief='groove')
        self.TopFrame.configure(borderwidth="2")
        self.TopFrame.configure(background=white_color)
        self.TopFrame.configure(width=585)

        self.TitleMainPageLabel = tk.Label(self.TopFrame, text="Hand Spasticty Assessment", font='Helvetica 18 bold')
        self.TitleMainPageLabel.place(relx=0.26, rely=0.08, relheight=0.2, relwidth=0.5)
        self.TitleMainPageLabel.configure(background=white_color)

        self.InfoLabel1 = tk.Label(self.TopFrame, text="To start a new assessment please create a new file and enter the patients infos",font='Helvetica 14' )
        self.InfoLabel1.place(relx=0.08, rely=0.33, relheight=0.2, relwidth=0.8)
        self.InfoLabel1.configure(background=white_color)

        self.InfoLabel2 = tk.Label(self.TopFrame, text="You can set the joints limits and the velocities on the Settings window",font='Helvetica 14')
        self.InfoLabel2.place(relx=0.02, rely=0.56, relheight=0.2, relwidth=0.9)
        self.InfoLabel2.configure(background=white_color)

    # Middle Frame
        self.MiddleFrame = tk.Frame(master)
        self.MiddleFrame.place(relx=0.008, rely=0.144, relheight=0.755, relwidth=0.977)
        self.MiddleFrame.configure(background=white_color)
        self.MiddleFrame.configure(borderwidth="2")
        self.MiddleFrame.configure(width=585)

        self.TardieuScaleTitleLabel = tk.Label(self.MiddleFrame, text="Tardieu Scale",font='Helvetica 20 bold')
        self.TardieuScaleTitleLabel.place(relx=0.272, rely=0.045, relheight=0.08, relwidth=0.5)
        self.TardieuScaleTitleLabel.configure(background=white_color)
        
        self.PatientAssesedLabel = tk.Label(self.MiddleFrame, text="Patient assessed : ",font='Helvetica 15 bold')
        self.PatientAssesedLabel.place(relx=0.008, rely=0.008, relheight=0.04, relwidth=0.21)
        self.PatientAssesedLabel.configure(background=white_color)

        self.PatientInfoLabel = tk.Label(self.MiddleFrame, text="",font='Helvetica 14', anchor=NW)
        self.PatientInfoLabel.place(relx=0.25, rely=0.008, relheight=0.04, relwidth=0.4)
        self.PatientInfoLabel.configure(background=white_color)
        self.PatientInfoLabel.configure(background="#f6f6f6")

        self.PressButtonLabel = tk.Label(self.MiddleFrame, text="(press buttons to start the assessment)", font='Helvetica 14 ')
        self.PressButtonLabel.place(relx=0.07, rely=0.11, relheight=0.04, relwidth=0.9)
        self.PressButtonLabel.configure(background=white_color)

        text_font = ('Helvetica', '16')
        self.comboExample = ttk.Combobox(self.MiddleFrame, state="readonly", font=text_font , width=5,justify='center',
                          values=["DIP joint", "PIP joint","MCP joint","All joints"])
        self.comboExample.Text = "All joints"
        self.comboExample.place(relx=0.42, rely=0.17, height=40, width=190)
        self.comboExample.bind("<<ComboboxSelected>>", self.selectedJoint)
        self.comboExample.set("DIP joint")

        self.V1Button = tk.Button(self.MiddleFrame, text="V1", font='Helvetica 16 bold', command=self.Assessment_V1_motor)
        self.V1Button.place(relx=0.1, rely=0.245, height=60, width=230)
        self.V1Button.configure(background="#1e8bc3")
        self.V1Button.configure(foreground=white_color)

        self.V2Button = tk.Button(self.MiddleFrame, text="V2", font='Helvetica 16 bold', command=self.Assessment_V2_motor)
        self.V2Button.place(relx=0.393, rely=0.245, height=60, width=230)
        self.V2Button.configure(background="#1e8bc3")
        self.V2Button.configure(foreground=white_color)
        self.V2Button.configure(takefocus="")

        self.V3Button = tk.Button(self.MiddleFrame, text="V3", font='Helvetica 16 bold', command=self.Assessment_V3_motor)
        self.V3Button.place(relx=0.687, rely=0.245, height=60, width=230)
        self.V3Button.configure(background="#1e8bc3")
        self.V3Button.configure(foreground=white_color)

        # Graph Presssure value
        self.GraphPressureSensorButton = tk.Button(self.MiddleFrame, text="Graph Tonus", font='Helvetica 16 ', command=self.plotPressureSensor)
        self.GraphPressureSensorButton.place(relx=0.41, rely=0.35, height=55, width=200)
        self.GraphPressureSensorButton.configure(background="#006666")
        self.GraphPressureSensorButton.configure(foreground=white_color)
        self.GraphPressureSensorButton.configure(background="#006666")

        self.SensorTitleAngleLabel = tk.Label(self.MiddleFrame, text="Force sensors",font='Helvetica 18 bold')
        self.SensorTitleAngleLabel.place(relx=0.28, rely=0.465, relheight=0.05, relwidth=0.5)
        self.SensorTitleAngleLabel.configure(background=white_color)
        
        self.SensorTopLabel = tk.Label(self.MiddleFrame, text="Top", font='Helvetica 16')
        self.SensorTopLabel.place(relx=0.068, rely=0.536, height=29, width=48)
        self.SensorTopLabel.configure(background=white_color)
        
        self.TopForceValue2 = 0
        self.SensorTopValue = tk.Label(self.MiddleFrame, text='', font='Helvetica 11')
        self.SensorTopValue.place(relx=0.79, rely=0.536, height=35, width=100)
        self.SensorTopValue.configure(background="#d9d9d9")
        self.SensorTopValue.after(200, self.refresh_top_sensor)        
        
        self.ProgressbarTop = ttk.Progressbar(self.MiddleFrame,orient ="horizontal",length = 200, value=0, mode ="determinate")
        self.ProgressbarTop.place(relx=0.2, rely=0.535, relheight=0.05, relwidth=0.55)
        self.ProgressbarTop.configure(maximum=500)
        self.ProgressbarTop.after(200, self.refresh_top_progress_bar)
         
        self.SensorBottomLabel = tk.Label(self.MiddleFrame, text="Bottom", font='Helvetica 16')
        self.SensorBottomLabel.place(relx=0.068, rely=0.615, height=29, width=75)
        self.SensorBottomLabel.configure(background=white_color)
        
        self.BottomForceValue2 = 0
        self.SensorBottomValue = tk.Label(self.MiddleFrame,  text='', font='Helvetica 11')
        self.SensorBottomValue.place(relx=0.79, rely=0.62, height=35, width=100)
        self.SensorBottomValue.configure(background="#d9d9d9") 
        self.SensorBottomValue.after(200, self.refresh_bottom_sensor)
        
        self.ProgressbarBottom = ttk.Progressbar(self.MiddleFrame,orient ="horizontal",value=0,length = 200, mode ="determinate")
        self.ProgressbarBottom.place(relx=0.2, rely=0.61, relheight=0.05, relwidth=0.55)
        self.ProgressbarBottom.configure(maximum=500)
        self.ProgressbarBottom.after(200, self.refresh_bottom_progress_bar)        
        
        self.SecondTitleAngleLabel = tk.Label(self.MiddleFrame, text="Angles of each hand-joint",font='Helvetica 18 bold')
        self.SecondTitleAngleLabel.place(relx=0.26, rely=0.7, relheight=0.05, relwidth=0.5)
        self.SecondTitleAngleLabel.configure(background=white_color)
          
        self.MCPAngleLabel = tk.Label(self.MiddleFrame, text="MCP", font='Helvetica 16')
        self.MCPAngleLabel.place(relx=0.068, rely=0.8, height=29, width=48)
        self.MCPAngleLabel.configure(background=white_color)

        self.PIPAngleLabel = tk.Label(self.MiddleFrame, text="PIP", font='Helvetica 16')
        self.PIPAngleLabel.place(relx=0.393, rely=0.8, height=29, width=41)
        self.PIPAngleLabel.configure(background=white_color)

        self.DIPAngleLabel = tk.Label(self.MiddleFrame, text="DIP", font='Helvetica 16')
        self.DIPAngleLabel.place(relx=0.704, rely=0.8, height=29, width=41)
        self.DIPAngleLabel.configure(background=white_color)
        
        self.PipAngleValue = tk.Label(self.MiddleFrame, font='Helvetica 16')
        self.PipAngleValue.place(relx=0.462, rely=0.785, height=51, width=100)
        self.PipAngleValue.configure(background="#d9d9d9")
        self.PipAngleValue.after(200, self.refresh_PipAngleValue)

        self.DipAngleValue = tk.Label(self.MiddleFrame, font='Helvetica 16')
        self.DipAngleValue.place(relx=0.769, rely=0.785, height=51, width=100)
        self.DipAngleValue.configure(background="#d9d9d9")
        self.DipAngleValue.after(200, self.refresh_DipAngleValue)

        self.McpAngleValue = tk.Label(self.MiddleFrame, font='Helvetica 16')
        self.McpAngleValue.place(relx=0.154, rely=0.785, height=51, width=100)
        self.McpAngleValue.configure(background="#d9d9d9")
        self.McpAngleValue.after(200, self.refresh_McpAngleValue)
    
        self.Degree1_label = tk.Label(self.MiddleFrame, text="o", font='Helvetica 16')
        self.Degree1_label.place(relx=0.276, rely=0.775, height=31, width=13)
        self.Degree1_label.configure(background=white_color)

        self.Degree2_label = tk.Label(self.MiddleFrame, text="o", font='Helvetica 16')
        self.Degree2_label.place(relx=0.584, rely=0.775, height=31, width=13)
        self.Degree2_label.configure(background=white_color)

        self.Degree3_label = tk.Label(self.MiddleFrame, text="o", font='Helvetica 16')
        self.Degree3_label.place(relx=0.892, rely=0.775, height=31, width=13)
        self.Degree3_label.configure(background=white_color)
        
        self.GraphAngleButton = tk.Button(self.MiddleFrame, text="Graph joint-angles", font='Helvetica 16 ',
                                             command=self.plotAngles)
        self.GraphAngleButton.place(relx=0.38, rely=0.89, height=55, width=230)
        self.GraphAngleButton.configure(background="#cc6600")
        self.GraphAngleButton.configure(foreground=white_color)

    # Bottom Frame
        self.BottomMainPageFrame = tk.Frame(master)
        self.BottomMainPageFrame.place(relx=0.008, rely=0.905, relheight=0.09, relwidth=0.977)
        self.BottomMainPageFrame.configure(relief='groove')
        self.BottomMainPageFrame.configure(borderwidth="2")
        self.BottomMainPageFrame.configure(background=white_color)

        self.StopMainPageButton = tk.Button(self.BottomMainPageFrame, text="STOP", font='Helvetica 16 bold ',
                                           command=self.functions.stop)
        self.StopMainPageButton.place(relx=0.020, rely=0.115, height=65, width=140)
        self.StopMainPageButton.configure(background="#d8163c")
        self.StopMainPageButton.configure(foreground=white_color) 
        
        self.TestsMainPageButton = tk.Button(self.BottomMainPageFrame, text="Tests",
                            command = self.technical_window, font='Helvetica 16 ')
        self.TestsMainPageButton.place(relx=0.300, rely=0.115, height=65, width=140)
        self.TestsMainPageButton.configure(background="#1abc9c")
        self.TestsMainPageButton.configure(foreground=white_color)

        self.SettingsMainPageButton = tk.Button(self.BottomMainPageFrame,text="Settings",
                            command = self.settings_window , font='Helvetica 16 ')
        self.SettingsMainPageButton.place(relx=0.480, rely=0.115, height=65, width=140)
        self.SettingsMainPageButton.configure(background="#1e8bc3")
        self.SettingsMainPageButton.configure(foreground=white_color) 

        self.SaveMainPageButton = tk.Button(self.BottomMainPageFrame,text="Save",
                            command = self.pdf_export , font='Helvetica 16 ')
        self.SaveMainPageButton.place(relx=0.660, rely=0.115, height=65, width=140)
        self.SaveMainPageButton.configure(background="#2ecc71")
        self.SaveMainPageButton.configure(foreground=white_color)		

        self.QuitMainPageButton = tk.Button(self.BottomMainPageFrame, text="Quit",
                            command= self.close_windows, font='Helvetica 16 ')
        self.QuitMainPageButton.place(relx=0.840, rely=0.115, height=65, width=140)
        self.QuitMainPageButton.configure(background="#666666")
        self.QuitMainPageButton.configure(foreground=white_color)

#----------------------------- Functions Main GUI ----------------------------- 
#------------------------------------------------------------------------------ 

# Functions for the menu 
    # Function to enter the firstname and lastname of the patient
    def patient_info(self):
        self.popUpInfo = Tk()
        self.popUpInfo.title("Informations about the patient :")
        self.popUpInfo.geometry("490x280+90+90")
        self.popUpInfo.configure(background=white_color)

        self.titleLabel = tk.Label(self.popUpInfo, text="Enter the patient informations", font='Helvetica 16')
        self.titleLabel.place(relx=0.08, rely=0.05, height=31, width=413)
        self.titleLabel.configure(background=white_color)

        self.FirstNameLabel = tk.Label(self.popUpInfo, text="First Name :", font='Helvetica 16')
        self.FirstNameLabel.place(relx=0.03, rely=0.25, height=31, width=213)
        self.FirstNameLabel.configure(background=white_color)

        self.FirstNameEntry = Entry(self.popUpInfo,width=10)
        self.FirstNameEntry.place(relx=0.5, rely=0.25, height=31, width=213)

        self.LastNameLabel = tk.Label(self.popUpInfo, text="Last Name :", font='Helvetica 16')
        self.LastNameLabel.place(relx=0.03, rely=0.45, height=31, width=213)
        self.LastNameLabel.configure(background=white_color)

        self.LastNameEntry = Entry(self.popUpInfo,width=10)
        self.LastNameEntry.place(relx=0.5, rely=0.45, height=31, width=213)

        self.SaveInfoButton = tk.Button(self.popUpInfo, text="Save", font='Helvetica 16 bold', command=self.SaveInfo)
        self.SaveInfoButton.place(relx=0.23, rely=0.7, height=50, width=120)
        self.SaveInfoButton.configure(background="#1e8bc3")
        self.SaveInfoButton.configure(foreground=white_color)

        self.QuitButtonInfo = tk.Button(self.popUpInfo, text="Quit", font='Helvetica 16 bold', command=self.close_info)
        self.QuitButtonInfo.place(relx=0.55, rely=0.7, height=50, width=120)
        self.QuitButtonInfo.configure(background="#666666")
        self.QuitButtonInfo.configure(foreground=white_color)

#Saving the infos of the patient
    def SaveInfo(self):
        global LastName
        global FirstName
        LastName = self.LastNameEntry.get()
        FirstName = self.FirstNameEntry.get()
        print("Last Name :" ,LastName, "and the first name",FirstName)
        self.PatientInfoLabel.configure(text="   %s  %s" %(LastName ,FirstName))
        self.close_info()

    def close_info(self):
        self.popUpInfo.destroy()

# Pop up window for About in Help 
    def about_window(self):
        messagebox.showinfo("Project information", "This project has been handled by a PhD student William McColl and a master student Djouzar ABASSEBAY. The aim is to assessment spastic patient using the Tardieu scale and a hand exoskeleton")

# Function for saving the selected joint and change the color of the button
    def selectedJoint(self, event):
        global selected_joint

        if self.comboExample.get() == "DIP joint":
            selected_joint = "DIP" 
            self.V1Button.configure(background="#1e8bc3")
            self.V2Button.configure(background="#f6f6f6")
            self.V3Button.configure(background="#1e8bc3")
            print ("DIP joint seleted")
            messagebox.showinfo("Assessment information", "You selected DIP joint, click OK to confirm")
        elif self.comboExample.get() == "PIP joint":
            selected_joint = "PIP"
            self.V1Button.configure(background="#1e8bc3")
            self.V2Button.configure(background="#f6f6f6")
            self.V3Button.configure(background="#1e8bc3")
            print ("PIP joint seleted")
            messagebox.showinfo("Assessment information", "You selected PIP joint, click OK to confirm")
        elif self.comboExample.get() == "MCP joint":
            selected_joint = "MCP"
            self.V1Button.configure(background="#1e8bc3")
            self.V2Button.configure(background="#f6f6f6")
            self.V3Button.configure(background="#1e8bc3")
            print ("MCP joint seleted")
            messagebox.showinfo("Assessment information", "You selected MCP joint, click OK to confirm")
        elif self.comboExample.get() == "All joints":
            selected_joint = "All"
            self.V1Button.configure(background="#f6f6f6")
            self.V2Button.configure(background="#1e8bc3")
            self.V3Button.configure(background="#f6f6f6")
            print ("All joint seleted")
            messagebox.showinfo("Assessment information", "You selected All joints, click OK to confirm")

# Function for the motor control for the V1 assessment 
    def Assessment_V1_motor (self):
        global time_assessment_begin
        global time_assessment_end
        global MCPjointValue
        global Pip_angle
        global Dip_angle
        global which_assessment

        global V1_dip_max_angle
        global V1_dip_min_angle
        global V1_dip_bottom_max_pressure
        global V1_dip_top_max_pressure
        
        global V1_pip_max_angle
        global V1_pip_min_angle
        global V1_pip_max_bottom_pressure
        global V1_pip_min_bottom_pressure
        
        global V1_mcp_max_angle
        global V1_mcp_min_angle
        global V1_mcp_bottom_max_pressure
        global V1_mcp_top_max_pressure

        which_assessment = 'V1'  # This values will be used to saved the plots 
        assessement = 'progress'
        connection.write('s'.encode())  # Send a letter to the arduino to light on the LED
        time_assessment_begin = time.time() - first_time   # Convert the time into seconds
        self.functions.stop()   # Initailize the motors
        time.sleep(0.5)

        # The DIP is selected in the Combobox
        if selected_joint == "DIP" : 
            print ("---------Assessment V1 start for DIP--------- ")
            print ("--------------------------------------------- ")
            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                    if (Dip_angle<100):
                        print(Dip_angle)
                        print ("1")
                        time.sleep(0.1)
                        Velocity_M1 = 1.7 if Dip_angle < 10 else 2.4
                        self.functions.CommandVel(2,Velocity_M1)
                        V1_dip_max_angle = Dip_angle
                        #print('0')
                    else :
                        print ("2")
                        self.functions.CommandVel(2,0)
                        time.sleep(2)
                        #print('1')
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                    time.sleep(0.1)
                                    print ("3")
                                    Velocity_M1 = 1.7 if Dip_angle < 8 else 2.4
                                    self.functions.CommandVel(2,-Velocity_M1)
                                    #print('2')
                                    #print(Dip_angle)
                                    if Dip_angle < 8 : 
                                        print ("4")
                                        assessement='finish'
                                        self.functions.CommandVel(1,0)
                                        self.functions.CommandVel(2,0)
                                        print('Threshold hit')
                                    else : 
                                        print ("5")
                                        assessement='progress'
                            else:
                                assessement='finish'
                                print ("6")
                                self.functions.CommandVel(1,0)
                                self.functions.CommandVel(2,0)
                else :
                    V1_dip_max_angle = Dip_angle
                    self.functions.CommandVel(2,0)
                    time.sleep(2)
                    print('7')
                    while (assessement != 'finish'):
                        if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                print ("8")
                                time.sleep(0.1)
                                Velocity_M1 = 1.7 if Dip_angle < 8 else 2.4
                                self.functions.CommandVel(2,-Velocity_M1)
                                #print('2')
                                #print(Dip_angle)
                                if Dip_angle < 8 : 
                                    print ("9")
                                    V1_dip_min_angle = Dip_angle
                                    assessement='finish'
                                    self.functions.CommandVel(1,0)
                                    self.functions.CommandVel(2,0)
                                    print('Threshold hit')
                                    #print('3')
                                else : 
                                    print ("10")
                                    assessement='progress'
                        else:
                            print ("11")
                            V1_dip_min_angle = Dip_angle
                            assessement='finish'
                            self.functions.CommandVel(1,0)
                            self.functions.CommandVel(2,0)
            time_assessment_end = time.time() - first_time
            assessement = 'progress'
            print ("angle bottom saved:",V1_dip_max_angle)
            print ("angle top saved:",V1_dip_min_angle)
            connection.write('e'.encode())
            print ("---------Assessment V1 is finished for DIP-------")

        elif selected_joint == "PIP" : 
            print ("---------Assessment V1 start for PIP--------- ")
            print ("--------------------------------------------- ")
            time_assessment_begin = time.time() - first_time
            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                    if (Pip_angle<50):
                        V1_pip_max_angle = Pip_angle
                        if (Dip_angle<12):
                            Velocity_M1 = 1.3 if Pip_angle < 10 else 1.4
                            Velocity_M2 = 2.4 if Pip_angle < 10 else 3.2
                            self.functions.CommandVel(2,Velocity_M1)
                            self.functions.CommandVel(1,Velocity_M2)
                        else:
                            Velocity_M2 = 2.4 if Pip_angle < 10 else 3.2
                            self.functions.CommandVel(2,0)
                            self.functions.CommandVel(1,Velocity_M2)
                            #print('0')
                    else:
                        print("first threshold reached")
                        self.functions.CommandVel(1,0)
                        self.functions.CommandVel(2,0)
                        time.sleep(2)
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                time.sleep(0.1)
                                Velocity_M1 = 1.8 if Pip_angle < 10 else 2.2
                                Velocity_M2 = 2 if Pip_angle < 10 else 2.4
                                self.functions.CommandVel(2,-Velocity_M1)
                                self.functions.CommandVel(1,-Velocity_M2)
                                #print('2')
                                #print(Dip_angle)
                                if (Pip_angle < 9 and Dip_angle < 9): 
                                    assessement='finish'
                                    self.functions.CommandVel(1,0)
                                    self.functions.CommandVel(2,0)
                                    #print('3')
                                else: 
                                    assessement='progress'
                            else:
                                assessement='finish'
                                self.functions.CommandVel(1,0)
                                self.functions.CommandVel(2,0)
                else:
                    V1_pip_max_angle = Pip_angle
                    print("first threshold reached")
                    self.functions.CommandVel(1,0)
                    self.functions.CommandVel(2,0)
                    time.sleep(2)
                    while (assessement != 'finish'):
                        if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                            time.sleep(0.1)
                            Velocity_M1 = 1.8 if Pip_angle < 10 else 2.2
                            Velocity_M2 = 2 if Pip_angle < 10 else 2.4
                            self.functions.CommandVel(2,-Velocity_M1)
                            self.functions.CommandVel(1,-Velocity_M2)
                            #print('2')
                            #print(Dip_angle)
                            if(Pip_angle < 9 and Dip_angle < 9):
                                V1_pip_min_angle = Pip_angle
                                assessement='finish'
                                self.functions.CommandVel(1,0)
                                self.functions.CommandVel(2,0)
                                #print('3')
                            else: 
                                assessement='progress'
                        else:
                            V1_pip_min_angle = Pip_angle
                            assessement='finish'
                            self.functions.CommandVel(1,0)
                            self.functions.CommandVel(2,0)
            time_assessment_end = time.time() - first_time
            connection.write('e'.encode())
            print ("angle min saved:",V1_pip_min_angle)
            print ("angle max saved:",V1_pip_max_angle)
            print ("---------Assessment V1 is finished for PIP-------")

        elif selected_joint == "MCP" : 
            print ("---------Assessment V1 start for MCP--------- ")
            print ("--------------------------------------------- ")
            time_assessment_begin = time.time() - first_time

            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold ):
                    if (MCPjointValue<85):
                        V1_mcp_max_angle = MCPjointValue
                        if (Dip_angle<10 and Pip_angle<10):
                            time.sleep(0.15)
                            Velocity_M1 = 1.3 if MCPjointValue < 10 else 1.6
                            Velocity_M2 = 1.9 if MCPjointValue < 10 else 2.4
                            Velocity_M3 = 1.7 if MCPjointValue < 10 else 2.4
                            self.functions.CommandVel(2,Velocity_M1)
                            self.functions.CommandVel(1,Velocity_M2)
                            self.functions.CommandVel(3,Velocity_M3)
                        else:
                            time.sleep(0.15)
                            Velocity_M3 = 2 if MCPjointValue < 10 else 1.8
                            self.functions.CommandVel(1,1.4)
                            self.functions.CommandVel(2,0)
                            self.functions.CommandVel(3,Velocity_M3)
                    else:
                        self.functions.CommandVel(1,0)
                        self.functions.CommandVel(2,0)
                        self.functions.CommandVel(3,0)
                        time.sleep(1)
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                time.sleep(0.1)
                                Velocity_M1 = 1.5 if MCPjointValue < 10 else 2
                                Velocity_M2 = 1.5 if MCPjointValue < 10 else 2
                                Velocity_M3 = 1.4 if MCPjointValue < 10 else 1.6
                                self.functions.CommandVel(2,-Velocity_M1)
                                self.functions.CommandVel(1,-Velocity_M2)
                                self.functions.CommandVel(3,-Velocity_M3)
                                if (MCPjointValue < 4):
                                    if (Pip_angle > 7):
                                        Velocity_M1 = 1.3 
                                        Velocity_M2 = 1.3 
                                        self.functions.CommandVel(2,-Velocity_M1)
                                        self.functions.CommandVel(1,-Velocity_M2)
                                        self.functions.CommandVel(3,0)
                                    elif (MCPjointValue < 4 and Dip_angle < 7 and Pip_angle < 7) : 
                                        assessement='finish'
                                        self.functions.CommandVel(1,0)
                                        self.functions.CommandVel(2,0)
                                        self.functions.CommandVel(3,0)
                                else : 
                                    assessement='progress'
                            else:
                                assessement='finish'
                                self.functions.CommandVel(1,0)
                                self.functions.CommandVel(2,0)
                                self.functions.CommandVel(3,0)
                else:
                    V1_mcp_max_angle = MCPjointValue
                    self.functions.CommandVel(1,0)
                    self.functions.CommandVel(2,0)
                    self.functions.CommandVel(3,0)
                    time.sleep(1)
                    while (assessement != 'finish'):
                        if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                            time.sleep(0.1)
                            Velocity_M1 = 1.6 if MCPjointValue < 10 else 2
                            Velocity_M2 = 1.6 if MCPjointValue < 10 else 2
                            Velocity_M3 = 1.4 if MCPjointValue < 10 else 1.6
                            self.functions.CommandVel(2,-Velocity_M1)
                            self.functions.CommandVel(1,-Velocity_M2)
                            self.functions.CommandVel(3,-Velocity_M3)
                            if (MCPjointValue < 4):
                                V1_mcp_min_angle = MCPjointValue
                                if (Pip_angle > 7):
                                    Velocity_M1 = 1.3 
                                    Velocity_M2 = 1.3 
                                    self.functions.CommandVel(2,-Velocity_M1)
                                    self.functions.CommandVel(1,-Velocity_M2)
                                    self.functions.CommandVel(3,0)
                                elif (MCPjointValue < 4 and Dip_angle < 7 and Pip_angle < 7) : 
                                    assessement='finish'
                                    self.functions.CommandVel(1,0)
                                    self.functions.CommandVel(2,0)
                                    self.functions.CommandVel(3,0)
                            else : 
                                assessement='progress'
                        else:
                            V1_mcp_min_angle = MCPjointValue
                            assessement='finish'
                            self.functions.CommandVel(1,0)
                            self.functions.CommandVel(2,0)
                            self.functions.CommandVel(3,0)
            time_assessment_end = time.time() - first_time
            connection.write('e'.encode())
            print("angle min saved:",V1_mcp_min_angle)
            print("angle max saved:",V1_mcp_max_angle)
            print("---------Assessment V1 is finished for MCP-------")

# This function can be used just to show up how the prototype is working 
        elif selected_joint == "All" :
            print ("Assessment V1 start for All ")
            print ("Assessment V2 start for DIP ")
            time_assessment_begin = time.time() - first_time
            time.sleep(5)
            self.functions.CommandVel(1,3.1)
            self.functions.CommandVel(2,3.6)
            time.sleep(1.5)
            self.functions.CommandVel(3,2.7)
            time.sleep(2)
            self.functions.CommandVel(1,0)
            self.functions.CommandVel(2,0)
            self.functions.CommandVel(3,0)
            time.sleep(2)
            self.functions.CommandVel(1,-3.1)
            self.functions.CommandVel(2,-3.6)
            time.sleep(1.5)
            self.functions.CommandVel(3,-2.7)
            time.sleep(2)
            self.functions.CommandVel(1,0)
            self.functions.CommandVel(2,0)
            self.functions.CommandVel(3,0)
            time_assessment_end = time.time() - first_time
            connection.write('e'.encode())
            print ("---------Assessment V1 is finished for All the joints-------")


# Function for the motor control for the V2 assessment 
    def Assessment_V2_motor (self):
        global time_assessment_begin
        global time_assessment_end
        global which_assessment

        self.functions.stop()
        time.sleep(0.5)
        which_assessment = 'V2'
        assessement = 'progress'
        var ="progress"
        #connection.write('s'.encode())
        if (selected_joint == "DIP" or selected_joint == "PIP" or selected_joint == "MCP" or selected_joint == "All"): 
            print ("------------ Assessment V2 start ------------ ")
            print ("--------------------------------------------- ")
            time_assessment_begin = time.time() - first_time
            time_test = time_assessment_begin
            while (time_test  < (time_assessment_begin+8)):
                time_test = time.time() - first_time
                print ("time:", time_test)
                print ("----------time_assessment_begin:",time_assessment_begin)
                print("0")
                time.sleep(0.1)
                if (int(BottomValue)<2):
                    self.functions.CommandVel(1,0)
                    self.functions.CommandVel(2,0)
                    self.functions.CommandVel(3,0)
                    print("-------1")
                    print(BottomValue)
                elif(int(BottomValue)>2 and int(BottomValue)<20):
                    Velocity_M1 = 1.6 
                    Velocity_M2 = 1.6 
                    Velocity_M3 = 1.5
                    print("-------------------------2")
                    self.functions.CommandVel(2,Velocity_M1)
                    self.functions.CommandVel(1,Velocity_M2)
                    self.functions.CommandVel(3,Velocity_M3)
                elif(int(BottomValue)>20 and int(BottomValue)<40):
                    Velocity_M1 = 1.9 
                    Velocity_M2 = 1.9 
                    Velocity_M3 = 1.7
                    print("-------------------------2")
                    self.functions.CommandVel(2,Velocity_M1)
                    self.functions.CommandVel(1,Velocity_M2)
                    self.functions.CommandVel(3,Velocity_M3)
                elif(int(BottomValue)>40 and int(BottomValue)<80):
                    Velocity_M1 = 2.2 
                    Velocity_M2 = 2.2 
                    Velocity_M3 = 2
                    print("-------------------------2")
                    self.functions.CommandVel(2,Velocity_M1)
                    self.functions.CommandVel(1,Velocity_M2)
                    self.functions.CommandVel(3,Velocity_M3)
                elif (int(BottomValue)>80):
                    Velocity_M1 = 2.5
                    Velocity_M2 = 2.5
                    Velocity_M3 = 2.2
                    print("-----------------------------------3")
                    self.functions.CommandVel(2,Velocity_M1)
                    self.functions.CommandVel(1,Velocity_M2)
                    self.functions.CommandVel(3,Velocity_M3)

            while(assessement != 'finish'):
                Velocity_M1 = 1.7
                Velocity_M2 = 1.7 
                Velocity_M3 = 1.3 
                self.functions.CommandVel(2,-Velocity_M1)
                self.functions.CommandVel(1,-Velocity_M2)
                self.functions.CommandVel(3,-Velocity_M3)
                print("7")
                while(assessement != 'finish'):
                    if (Pip_angle< 10):
                        self.functions.CommandVel(2,0)
                        self.functions.CommandVel(1,0)
                        self.functions.CommandVel(3,0)
                        time.sleep(2)
                        print("stop")
                        assessement = 'finish'
            time_assessment_end = time.time() - first_time
            print ("---------Assessment V2 is finished-------")
            assessement = 'progress'
            connection.write('e'.encode())

# Function for the motor control for the V3 assessment 
    def Assessment_V3_motor (self):
        global time_assessment_begin
        global time_assessment_end
        global which_assessment
        global MCPjointValue
        global Pip_angle
        global Dip_angle
        global which_assessment

        global V1_dip_max_angle
        global V1_dip_min_angle
        global V1_pip_max_angle
        global V1_pip_min_angle
        global V1_mcp_max_angle
        global V1_mcp_min_angle
        
        which_assessment = 'V3'
        assessement = 'progress'
        connection.write('s'.encode())
        self.functions.stop()
        time.sleep(0.3)
        # The DIP is selected in the Combobox
        if selected_joint == "DIP" : 
            time_assessment_begin = time.time() - first_time
            print ("---------Assessment V3 start for DIP--------- ")
            print ("--------------------------------------------- ")
            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold) :
                    if (Dip_angle<V1_dip_max_angle):
                        time.sleep(0.01)
                        Velocity_M1 = 3 if Dip_angle < 10 else 9
                        self.functions.CommandVel(2,Velocity_M1)
                    else:
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                time.sleep(0.01)
                                Velocity_M1 = 3 if Dip_angle < 10 else 9
                                self.functions.CommandVel(2,-Velocity_M1)
                                if Dip_angle < V1_dip_min_angle : 
                                    assessement='finish'
                                    self.functions.CommandVel(2,0)
                                else : 
                                    assessement='progress'
                            else :
                                # If the threshhold is reached
                                self.functions.stop()
                                messagebox.showwarning("Warning", "The threshold has been reached, please restart the DIP assessment at V3")
                                break
                else :
                    self.functions.stop()
                    messagebox.showwarning("Warning", "The threshold has been reached, please restart the DIP assessment at V3")
                    break
            time_assessment_end = time.time() - first_time
            assessement = 'progress'
            connection.write('e'.encode())
            print ("---------Assessment V3 is finished for DIP-------")

        elif selected_joint == "PIP" : 
            print ("---------Assessment V3 start for PIP--------- ")
            print ("--------------------------------------------- ")
            time_assessment_begin = time.time() - first_time
            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold) :
                    if (Pip_angle<50):
                        time.sleep(0.1)
                        if (Dip_angle<4):
                            Velocity_M1 = 1.8 if Pip_angle < 15 else 2
                            Velocity_M2 = 4.4 if Pip_angle < 15 else 9
                            self.functions.CommandVel(2,Velocity_M1)
                            self.functions.CommandVel(1,Velocity_M2)
                        else:
                            Velocity_M2 = 4.4 if Pip_angle < 15 else 8
                            self.functions.CommandVel(2,0)
                            self.functions.CommandVel(1,Velocity_M2)
                    else:
                        self.functions.CommandVel(1,0)
                        self.functions.CommandVel(2,0)
                        time.sleep(1.2)
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                time.sleep(0.1)
                                Velocity_M1 =  2.5 if Pip_angle < 15 else 3
                                Velocity_M2 = 4.4 if Pip_angle < 15 else 9
                                self.functions.CommandVel(2,-Velocity_M1)
                                self.functions.CommandVel(1,-Velocity_M2)
                                if Pip_angle < 4 : 
                                    assessement='finish'
                                    self.functions.CommandVel(1,0)
                                    self.functions.CommandVel(2,0)
                                else : 
                                    assessement='progress'
                            else :
                                # If the threshhold is reached
                                self.functions.stop()
                                messagebox.showwarning("Warning", "The threshold has been reached, please restart the PIP assessment at V3")
                                break
                else :
                    self.functions.stop()
                    messagebox.showwarning("Warning", "The threshold has been reached, please restart the PIP assessment at V3")
                    break
            time_assessment_end = time.time() - first_time
            connection.write('e'.encode())
            print ("---------Assessment V3 is finished for DIP-------")

        elif selected_joint == "MCP" : 
            print ("---------Assessment V3 start for MCP--------- ")
            print ("--------------------------------------------- ")
            time_assessment_begin = time.time() - first_time
            while (assessement != 'finish'):
                if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold) :
                    if (MCPjointValue<82):
                        if (Dip_angle<7):
                            print(MCPjointValue)
                            time.sleep(0.2)
                            Velocity_M1 = 3 if MCPjointValue < 10 else 4
                            Velocity_M2 = 3 if MCPjointValue < 10 else 4
                            Velocity_M3 = 4 if MCPjointValue < 10 else 10
                            self.functions.CommandVel(2,Velocity_M1)
                            self.functions.CommandVel(1,Velocity_M2)
                            self.functions.CommandVel(3,Velocity_M2)
                        else:
                            time.sleep(0.2)
                            Velocity_M3 = 4 if MCPjointValue < 10 else 8
                            self.functions.CommandVel(1,0)
                            self.functions.CommandVel(2,0)
                            self.functions.CommandVel(3,Velocity_M2)
                    else:
                        self.functions.stop()
                        time.sleep(2)
                        while (assessement != 'finish'):
                            if (int(TopValue) < top_sensor_threshold and int(BottomValue) < bottom_sensor_threshold):
                                time.sleep(0.2)
                                Velocity_M1 = 3 if MCPjointValue < 10 else 8.5
                                Velocity_M2 = 3 if MCPjointValue < 10 else 8.5
                                Velocity_M3 = 3 if MCPjointValue < 10 else 10
                                self.functions.CommandVel(2,-Velocity_M1)
                                self.functions.CommandVel(1,-Velocity_M2)
                                self.functions.CommandVel(3,-Velocity_M2)
                                if MCPjointValue < 12 : 
                                    assessement='finish'
                                    self.functions.stop()
                                else : 
                                    assessement='progress'
                            else :
                                # If the threshhold is reached
                                self.functions.stop()
                                messagebox.showwarning("Warning", "The threshold has been reached, please restart the PIP assessment at V3")
                                break
                else :
                    self.functions.stop()
                    messagebox.showwarning("Warning", "The threshold has been reached, please restart the PIP assessment at V3")
                    break
            time_assessment_end = time.time() - first_time
            connection.write('e'.encode())
            print ("---------Assessment V3 is finished for MCP-------")

# Function to plot the force sensors values 
    def plotPressureSensor(self):
        global time_assessment_begin
        global time_assessment_end
        global max_top
        global max_bottom
        max_bottom = 0
        max_top = 0
        min_bottom = 0
        min_top = 0
        max_plot_pressure = 0
        min_plot_pressure = 0
        now_plot = []
        bottom_value_plot = []
        top_value_plot = []

        # Only take the values during the start of the assessment and the end (when we plot)
        for i in range (len(now)):
            if (now[i]>time_assessment_begin and now[i]<time_assessment_end):
                top_value_plot.append(top_sensor_array[i-1])
                bottom_value_plot.append(bottom_sensor_array[i-1])
                now_plot.append(now[i])

        # Max and min values for plots and informations
        max_time = np.amax(np.array(now_plot)) 
        min_time = np.min(np.array(now_plot)) 
        max_top = np.amax(np.array(top_sensor_array).astype(np.float))  
        min_top = np.min(np.array(top_sensor_array).astype(np.float))  
        max_bottom = np.amax(np.array(bottom_value_plot).astype(np.float))  
        min_bottom = np.min(np.array(bottom_value_plot).astype(np.float))  
        max_dip_angle = np.amax(np.array(Pip_angle_array))
        max_pip_angle = np.amax(np.array(Dip_angle_array))
        try:
            max_mcp_angle = np.amax(MCPjointValue_array)
        except ValueError:
            pass

        # For a better plot resize the window
        if max_top < max_bottom : 
            max_plot_pressure = max_bottom;
            min_plot_pressure = min_bottom
        else :
            max_plot_pressure = max_top;
            min_plot_pressure = min_top

        # Apply a light savgol filter to smooth the values 
        bottom_value_plot_filtered = signal.savgol_filter(bottom_value_plot, 5, 3)
        top_value_plot_filtered = signal.savgol_filter(top_value_plot, 5, 3)

        # Diplay the max values in the terminal 
        print("\n ----------------- Assessment informations ----------------- \n ")
        print ("The maximum value for the top sensor is :", max_bottom)
        print ("The maximum value for the bottom  sensor is :", max_top)

        # Create a window and plot the values
        plt.figure("Pressure Sensors (top and bottom)")
        plt.axis([min_time-1, max_time+1, min_plot_pressure, 1.02*max_plot_pressure])
        plt.plot(now_plot,bottom_value_plot_filtered,color='red')
        plt.plot(now_plot,top_value_plot_filtered,color='green')
        #plt.plot(now_plot,bottom_value_plot,color='yellow')
        #plt.plot(now_plot,top_value_plot,color='yellow')
        #plt.plot(now_plot,bottom_value_plot,color='purple')
        #plt.plot(now_plot,top_value_plot,color='yellow')
       
        plt.ylabel("Pressure (Newton)", fontsize=14)
        plt.xlabel("Time (s)", fontsize=14)

        if (which_assessment == 'V1'):
            if (selected_joint == 'DIP'):
                plt.savefig('Pressure_V1_DIP.png')
                plt.suptitle ('Plot of the force sensors values for DIP at V1 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
            elif (selected_joint == 'PIP'):
                plt.savefig('Pressure_V1_PIP.png')
                plt.suptitle ('Plot of the force sensors values for PIP at V1 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
            elif (selected_joint == 'MCP'):
                plt.savefig('Pressure_V1_MCP.png')
                plt.suptitle ('Plot of the force sensors values for MCP at V1 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
        elif (which_assessment == 'V1'):
                plt.savefig('Pressure_V2.png')
                plt.suptitle ('Plot of the force sensors values at V2 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
        elif (which_assessment =='V3'):
            if (selected_joint == 'DIP'):
                plt.savefig('Pressure_V3_DIP.png')
                plt.suptitle ('Plot of the force sensors values for DIP at V3 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
            elif (selected_joint == 'PIP'):
                plt.savefig('Pressure_V3_PIP.png')
                plt.suptitle ('Plot of the force sensors values for PIP at V3 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
            elif (selected_joint == 'MCP'):
                plt.savefig('Pressure_V3_MCP.png')
                plt.suptitle ('Plot of the force sensors values for MCP at V3 \nTop Sensor (red) , Bottom Sensor (green)', fontsize=16)
        plt.show()

        del top_value_plot[:]
        del bottom_value_plot[:]
        del now_plot[:]

# Plot angles of the MCP, DIP and PIP joints
    def plotAngles(self):
        max_time_angles = 0
        min_time_angles = 0
        max_value_angles = 0
        min_value_angles = 0
        now_plot_angle = []
        now_plot_angle2 = []
        pip_angle_plot = []
        dip_angle_plot = []
        MCP_angle_plot_array = []
        global time_assessment_begin
        global time_assessment_end

        # Only take the values during the start of the assessment and the end (when we plot)
        for i in range (len(TimeJointValues_array)):
            if (TimeJointValues_array[i]>time_assessment_begin and TimeJointValues_array[i]<time_assessment_end):
                MCP_angle_plot_array.append(MCPjointValue_array[i])
                now_plot_angle.append(TimeJointValues_array[i])

        for i in range (len(now)): 
            if (now[i]>time_assessment_begin and now[i]<time_assessment_end):
                pip_angle_plot.append(Pip_angle_array[i-1])
                dip_angle_plot.append(Dip_angle_array[i-1])
                now_plot_angle2.append(now[i])

        # For a better plot resize the window
        max_time_angles = np.amax(now_plot_angle)
        min_time_angles = np.min(now_plot_angle)
        max_value_MCP = np.amax(MCP_angle_plot_array)
        max_value_dip = np.amax(dip_angle_plot)
        max_value_pip = np.amax(pip_angle_plot)
        min_value_MCP = np.min(MCP_angle_plot_array)
        min_value_dip = np.min(dip_angle_plot)
        min_value_pip = np.min(pip_angle_plot)
        max_value_angles = max(max_value_MCP, max_value_dip, max_value_pip)
        min_value_angles = min(min_value_MCP, min_value_dip, min_value_pip)

        print ("The maximum angle for : the DIP joint :", max_value_dip)
        print ("                        the PIP joint :", max_value_pip)
        print ("                        the MCP joint :", max_value_MCP)
        # Apply a filter to the values
        MCPjointValue_array_filtered = signal.savgol_filter(MCP_angle_plot_array, 31, 3)
        pip_angle_plot_filtered = signal.savgol_filter(pip_angle_plot, 11, 3)
        dip_angle_plot_filtered = signal.savgol_filter(dip_angle_plot, 9, 3)

        # Diplay the max values in the terminal 
        #max_range_angle = np.amax(MCPjointValue_array_filtered)
        #print ("The maximum range of motion for the MCP joint is : %.2f" % max_range_angle)

        plt.figure('Angles at V1')
        plt.axis([ min_time_angles-1,max_time_angles+1, min_value_angles, 1.02*max_value_angles])
        plt.plot(now_plot_angle, MCPjointValue_array_filtered,color='red')
        plt.plot(now_plot_angle2, pip_angle_plot_filtered,color='green')
        plt.plot(now_plot_angle2, dip_angle_plot_filtered,color='blue')
        plt.ylabel("Anlges (in degrees)", fontsize=14)
        plt.xlabel("Time (s)", fontsize=14)

        if (which_assessment == 'V1'):
            if (selected_joint == 'DIP'):
                plt.savefig('V1_DIP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for DIP at V1", fontsize=16)
            elif (selected_joint == 'PIP'):
                plt.savefig('V1_PIP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for PIP at V1", fontsize=16)
            elif (selected_joint == 'MCP'):
                plt.savefig('V1_MCP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for MCP at V1", fontsize=16)
        elif (which_assessment == 'V1'):
                plt.savefig('V2.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle at V2", fontsize=16)
        elif (which_assessment =='V3'):
            if (selected_joint == 'DIP'):
                plt.savefig('V3_DIP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for DIP at V3", fontsize=16)
            elif (selected_joint == 'PIP'):
                plt.savefig('V3_PIP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for PIP at V3", fontsize=16)
            elif (selected_joint == 'MCP'):
                plt.savefig('V3_MCP.png')
                plt.suptitle ("MCP (Red), PIP (Green) and DIP (Blue) angle for MCP at V3", fontsize=16)
        plt.show()

        del MCP_angle_plot_array[:]
        del now_plot_angle[:]
        del pip_angle_plot[:]
        del dip_angle_plot[:]
        del now_plot_angle2[:]

# Refesh the dsiplay of the values of force sensors 
    def refresh_top_sensor(self):
        self.SensorTopValue.configure(text="%s " % TopValue)
        self.SensorTopValue.after(100, self.refresh_top_sensor)

    def refresh_bottom_sensor(self):
        self.SensorBottomValue.configure(text="%s " % BottomValue)
        self.SensorBottomValue.after(100, self.refresh_bottom_sensor) 

# Refresh the progress bars
    def refresh_top_progress_bar(self):        
        NewTopValue = int(TopValue)
        self.ProgressbarTop.configure(value= NewTopValue)      
        self.ProgressbarTop.after(100, self.refresh_top_progress_bar)
        
    def refresh_bottom_progress_bar(self):    
        NewBottomValue = int(BottomValue)
        self.ProgressbarBottom.configure(value=  NewBottomValue)      
        self.ProgressbarBottom.after(100, self.refresh_bottom_progress_bar)

# Refresh the angles values
    def refresh_McpAngleValue(self):
        self.McpAngleValue.configure(text="%.i " % MCPjointValue)
        self.McpAngleValue.after(100, self.refresh_McpAngleValue)

    def refresh_PipAngleValue(self):
        self.PipAngleValue.configure(text="%s" % Pip_angle)
        self.PipAngleValue.after(100, self.refresh_PipAngleValue)

    def refresh_DipAngleValue_Test(self):
        global Dip_angle
        self.DipAngleValue.configure(text="%s" % Dip_angle)
        print("refreshed")

    def refresh_DipAngleValue(self):
        self.DipAngleValue.configure(text="%s" % Dip_angle)
        self.DipAngleValue.after(100, self.refresh_DipAngleValue)

# Create the settings and technical window (in order to create new windows)
    def settings_window(self):
        self.SettingsWindow = tk.Toplevel(self.master)
        self.app = SettingsPage(self.SettingsWindow)

    def technical_window(self):
        self.TechnicalWindow = tk.Toplevel(self.master)
        self.app = TechnicalPage(self.TechnicalWindow)

# Create a pdf file with all the information gathered
    def pdf_export (self):
        pdf=FPDF(format='letter', unit='in')
        pdf.add_page()
        pdf.set_font('Times','',10.0) 
        epw = pdf.w - 2*pdf.l_margin
        col_width = epw/4
        now = datetime.now() # current date and time
        year = now.strftime("%Y")
        month = now.strftime("%m")
        day = now.strftime("%d")
        time = now.strftime("%H:%M:%S")

        V1_assessment_data = [['Joint','Range of Motion (in °)',"Pressure Force"],
                              ['MCP'  ,[V1_mcp_min_angle, V1_mcp_max_angle],"    346"       ],
                              ['PIP'  ,[V1_pip_min_angle, V1_pip_max_angle],"    629"       ],
                              ['DIP'  ,[V1_dip_min_angle, V1_dip_max_angle] ,"    287"       ]]
        V2_assessment_data = [['Joint','Range of Motion',"Pressure Force"],
                              ['MCP',"58°","623"]]
        V3_assessment_data = [['Joint','Range of Motion',"Pressure Force"],
                              ['MCP',[V1_mcp_min_angle, V1_mcp_max_angle],"363"],
                              ['PIP',[V1_pip_min_angle, V1_pip_max_angle],"575"],
                              ['DIP',[V1_dip_min_angle, V1_dip_max_angle],"765"]]

        # Document title centered, 'B'old, 14 pt
        pdf.set_font('Times','B',14.0) 
        pdf.cell(epw, 0.0, 'Recap of the assessment of the patient', align='C')
        pdf.ln(0.5)

        try:
            pdf.cell(epw, 0.0, "Patient Last Name : %s" % LastName, align='C')
            pdf.ln(0.5)
            pdf.cell(epw, 0.0, "Patient First Name : %s" % FirstName, align='C')
            pdf.ln(0.5)
        except NameError:
            messagebox.showwarning("Error", "You forgot to enter the User information or the info have not been saved")
            print("You forgot to enter the User information or the info have not been saved")

        pdf.cell(epw, 0.0, "Date : %s / %s / %s and time : %s" % (day,month,year,time), align='C')
        pdf.ln(0.5)
        pdf.set_font('Times','',14.0) 

        # Text height is the same as current font size
        th = pdf.font_size
        pdf.ln(0.5)
        pdf.cell(epw, 0.0, "Velocity V1", align='L')
        pdf.ln(0.5)
        for row in V1_assessment_data:
            for datum in row:
                pdf.cell(col_width, 2*th, str(datum), border=1)
            pdf.ln(2*th)
        pdf.ln(0.5)
        pdf.cell(epw, 0.0, "Velocity V2", align='L')
        pdf.ln(0.5)
        for row in V2_assessment_data:
            for datum in row:
                pdf.cell(col_width, 2*th, str(datum), border=1)
            pdf.ln(2*th)
        pdf.ln(0.5)
        pdf.cell(epw, 0.0, "Velocity V3", align='L')
        pdf.ln(0.5)
        for row in V3_assessment_data:
            for datum in row:
                pdf.cell(col_width, 2*th, str(datum), border=1)
            pdf.ln(2*th)

# Try to save the saved plots into the pdf file if it exist

        try:
            pdf.image('V1_DIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "DIP angle at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V1_DIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V1_PIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "PIP angle at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V1_PIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V1_MCP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "MCP angle at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V1_MCP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V2.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Velocity 2", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V2.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V3_DIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "DIP angle at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V3_DIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V3_PIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "PIP angle at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V3_PIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('V3_MCP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "MCP angle at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'V3_MCP.png' is incorrect or not correctly saved")
            pass

# Try to save the Pressure plots 
        try:
            pdf.image('Pressure_V1_DIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V1_DIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V1_PIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors  at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V1_PIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V1_MCP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors  at Velocity 1", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V1_MCP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V2.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors at Velocity 2", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V2.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V3_DIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V3_DIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V3_PIP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V3_PIP.png' is incorrect or not correctly saved")
            pass

        try:
            pdf.image('Pressure_V3_MCP.png', w=pdf.w/2.0, h=pdf.h/4.0)
            pdf.ln(0.1)
            pdf.cell(epw, 0.0, "Pressure sensors at Velocity 3", align='L')
            pdf.ln(0.5)
        except OSError as e:
            print("'Pressure_V3_MCP.png' is incorrect or not correctly saved")
            pass
        print ("The assessment has been saved")
        pdf.output('Assessment.pdf','F')

# End the simulation 
    def close_windows(self):
        self.master.destroy()

'''---------------------------------- Tests page ----------------------------------'''
class TechnicalPage:
#---------------------------------- Tests GUI ---------------------------------
    def __init__(self, technical):
        self.functions = AllFunction()
        self.technical = technical
        self.frame = tk.Frame(self.technical)
        self.technical.geometry("600x423+420+120")
        self.technical.title("Tests")
        self.technical.configure(background="#d9d9d9")

    # Top Frame
        self.TopFrameTehnical = tk.Frame(technical)
        self.TopFrameTehnical.place(x=10, y=10, height=55, width=575)
        self.TopFrameTehnical.configure(relief='groove')
        self.TopFrameTehnical.configure(borderwidth="2")
        self.TopFrameTehnical.configure(background="#ffffff")

        self.SerialConnectionLabel = tk.Label(self.TopFrameTehnical, text="Serial Connection :", font='Helvetica 13')
        self.SerialConnectionLabel.place(x=70, y=10,  height=31, width=200)
        self.SerialConnectionLabel.configure(background="#ffffff")

        self.EstablishedLabel = tk.Label(self.TopFrameTehnical)
        self.EstablishedLabel.place(x=310, y=10,  height=31, width=130)
        self.EstablishedLabel.configure(background="#d9d9d9")
        self.Connection_check()
        

    # Middle Frame
        self.MiddleFrameTehnical = tk.Frame(technical)
        self.MiddleFrameTehnical.place(x=10, y=65, height=185, width=575)
        self.MiddleFrameTehnical.configure(relief='groove')
        self.MiddleFrameTehnical.configure(borderwidth="2")
        self.MiddleFrameTehnical.configure(background="#ffffff")
        self.MiddleFrameTehnical.configure(width=575)

        self.SliderMCPMotorSpeed = tk.Scale(self.MiddleFrameTehnical, from_=-10.0, to=10.0, resolution=0.01)
        self.SliderMCPMotorSpeed.place(x=180, y=20,height=48, width=240)
        self.SliderMCPMotorSpeed.configure(background="#ffffff")
        self.SliderMCPMotorSpeed.configure(highlightbackground="#ffffff")
        self.SliderMCPMotorSpeed.configure(orient="horizontal")
        
        self.SliderPIPMotorSpeed = tk.Scale(self.MiddleFrameTehnical, from_=-10.0, to=10.0, resolution=0.01)
        self.SliderPIPMotorSpeed.place(x=180, y=70,height=48, width=240)
        self.SliderPIPMotorSpeed.configure(background="#ffffff")
        self.SliderPIPMotorSpeed.configure(highlightbackground="#ffffff")
        self.SliderPIPMotorSpeed.configure(orient="horizontal")

        self.SliderDIPMotorSpeed = tk.Scale(self.MiddleFrameTehnical, from_=-10.0, to=10.0,resolution=0.01)
        self.SliderDIPMotorSpeed.place(x=180, y=120,height=52, width=242)
        self.SliderDIPMotorSpeed.configure(background="#ffffff")
        self.SliderDIPMotorSpeed.configure(highlightbackground="#ffffff")
        self.SliderDIPMotorSpeed.configure(orient="horizontal")
        
        self.MCPMotorSpeedLabel = tk.Label(self.MiddleFrameTehnical, text="DIP MotorSpeed", font='Helvetica 13')
        self.MCPMotorSpeedLabel.place(x=10, y=30,  height=31, width=148)
        self.MCPMotorSpeedLabel.configure(background="#ffffff")
        
        self.PIPMotorSpeedLabel = tk.Label(self.MiddleFrameTehnical, text="PIP MotorSpeed", font='Helvetica 13')
        self.PIPMotorSpeedLabel.place(x=10, y=80,  height=31, width=136)
        self.PIPMotorSpeedLabel.configure(background="#ffffff")
        
        self.DIPMotorSpeedLabel = tk.Label(self.MiddleFrameTehnical, text="MCP Motor Speed", font='Helvetica 13')
        self.DIPMotorSpeedLabel.place(x=10, y=130,  height=31, width=144)
        self.DIPMotorSpeedLabel.configure(background="#ffffff")

        self.OnMCPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Apply", command=self.MCPSpeedControl)
        self.OnMCPRadioButton.place(x=440, y=30, height=37, width=58)
        self.OnMCPRadioButton.configure(background="#d9d9d9")
        
        self.OFFMCPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Stop", command=self.stopMCP)
        self.OFFMCPRadioButton.place(x=500, y=30, height=37, width=63)
        self.OFFMCPRadioButton.configure(background="#d8163c")
        self.OFFMCPRadioButton.configure(foreground=white_color)
        
        self.OnPIPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Apply", command=self.PIPSpeedControl)
        self.OnPIPRadioButton.place(x=440, y=80, height=37, width=58)
        self.OnPIPRadioButton.configure(background="#d9d9d9")
        
        self.OFFPIPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Stop", command=self.stopPIP)
        self.OFFPIPRadioButton.place(x=500, y=80, height=37, width=63)
        self.OFFPIPRadioButton.configure(background="#d8163c")
        self.OFFPIPRadioButton.configure(foreground=white_color)
        
        self.OnDIPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Apply", command=self.DIPSpeedControl)
        self.OnDIPRadioButton.place(x=440, y=130, height=37, width=58)
        self.OnDIPRadioButton.configure(background="#d9d9d9")
        
        self.OffDIPRadioButton = tk.Button(self.MiddleFrameTehnical, text="Stop", command=self.stopDIP)
        self.OffDIPRadioButton.place(x=500, y=130, height=37, width=63)
        self.OffDIPRadioButton.configure(background="#d8163c")
        self.OffDIPRadioButton.configure(foreground=white_color)

# Pressure sensor tests Frame
        self.SensorFrame = tk.Frame(technical)
        self.SensorFrame.place(x=10, y=250, height=95, width=575)
        self.SensorFrame.configure(relief='groove')
        self.SensorFrame.configure(borderwidth="2")
        self.SensorFrame.configure(background="#ffffff")
        self.SensorFrame.configure(width=575)
        
        self.TopForceSensor = tk.Label(self.SensorFrame, text= "Top Force sensor", font='Helvetica 13')
        self.TopForceSensor.place(x=70, y=10,  height=31, width=167)
        self.TopForceSensor.configure(background="#ffffff")
     
    # Bottom Frame   
        self.BottomForceSensorLabel = tk.Label(self.SensorFrame, text="Bottom Force sensor", font='Helvetica 13')
        self.BottomForceSensorLabel.place(x=80, y=50,  height=31, width=177)
        self.BottomForceSensorLabel.configure(background="#ffffff")
        
        self.TopForceValue = 0
        self.TopForceSensorValue = tk.Label(self.SensorFrame, text='', font='Helvetica 11')
        self.TopForceSensorValue.place(x=310, y=10,  height=31, width=97)
        self.TopForceSensorValue.configure(background="#d9d9d9")
        self.TopForceSensorValue.after(200, self.refresh_top_sensor)
            
        self.BottomForceValue = 0
        self.BottomForceSensorValue = tk.Label(self.SensorFrame,  text='', font='Helvetica 11')
        self.BottomForceSensorValue.place(x=310, y=50,  height=31, width=97)
        self.BottomForceSensorValue.configure(background="#d9d9d9")
        self.BottomForceSensorValue.after(200, self.refresh_bottom_sensor)
                                         
        self.BottomFrameTechnical = tk.Frame(technical)
        self.BottomFrameTechnical.place(x=10, y=345, height=65, width=575)
        self.BottomFrameTechnical.configure(relief='groove')
        self.BottomFrameTechnical.configure(borderwidth="2")
        self.BottomFrameTechnical.configure(background="#ffffff")

        self.ResetTechnicalButton = tk.Button(self.BottomFrameTechnical, text="Reset", font='Helvetica 11', command=self.reset)
        self.ResetTechnicalButton.place(x=140, y=10,  height=42, width=78)
        self.ResetTechnicalButton.configure(background="#f15a22")
        self.ResetTechnicalButton.configure(foreground=white_color)
        
        self.StopTechnicalButton = tk.Button(self.BottomFrameTechnical, text="Stop", command=self.stopAll, font='Helvetica 11')
        self.StopTechnicalButton.place(x=260, y=10,  height=42, width=78)
        self.StopTechnicalButton.configure(background="#d8163c")
        self.StopTechnicalButton.configure(foreground=white_color)
        
        self.BackTechnicalButton = tk.Button(self.BottomFrameTechnical, text= "Quit", command = self.close_windows, font='Helvetica 11')
        self.BackTechnicalButton.place(x=380, y=10,  height=42, width=81)
        self.BackTechnicalButton.configure(background="#666666")
        self.BackTechnicalButton.configure(foreground=white_color)

#----------------------------- Functions Tests GUI ----------------------------
# Check the connection of the motors
    def Connection_check(self): 
        x=True
 #       x = rospy.Subscriber("/motor_states/pan_tilt_port", String)
        if (x) :
            self.EstablishedLabel.configure(foreground="#00cc00")
            self.EstablishedLabel.configure(text="Established")
            self.EstablishedLabel.configure(font='Helvetica 14')
        else:
            self.EstablishedLabel.configure(foreground="#ff0000")
            self.EstablishedLabel.configure(text="Disconnected")
            self.EstablishedLabel.configure(font='Helvetica 12')

# Refesh the dsiplay of the value of force sensors 
    def refresh_top_sensor(self):
        self.TopForceSensorValue.configure(text="%s " % TopValue)
        self.TopForceSensorValue.after(200, self.refresh_top_sensor)

    def refresh_bottom_sensor(self):
        self.BottomForceSensorValue.configure(text="%s " % BottomValue)
        self.BottomForceSensorValue.after(200, self.refresh_bottom_sensor) 

# Control the speed of the motors
    def MCPSpeedControl(self):
        if __name__ == '__main__':
            MCP_Velocity_value = self.SliderMCPMotorSpeed.get()
            print ("MCP joint speed :" ,self.SliderMCPMotorSpeed.get())
            self.functions.CommandVel(2,MCP_Velocity_value)

    def PIPSpeedControl(self):
        if __name__ == '__main__':
            PIP_Velocity_value = self.SliderPIPMotorSpeed.get()
            print ("PIP joint speed :" ,self.SliderPIPMotorSpeed.get())
            self.functions.CommandVel(1,PIP_Velocity_value)

    def DIPSpeedControl(self):
        if __name__ == '__main__':
            DIP_Velocity_value = self.SliderDIPMotorSpeed.get()
            print ("DIP joint speed :" ,self.SliderDIPMotorSpeed.get())
            self.functions.CommandVel(3,DIP_Velocity_value)

# Stop the motors        
    def stopAll(self):
        self.functions.CommandVel(1,0.00)
        self.functions.CommandVel(2,0.00)
        self.functions.CommandVel(3,0.00)
        print ("All joints stopped")

    def stopMCP(self):
        self.functions.CommandVel(2,0.00)
        print ("MCP joint stopped")

    def stopPIP(self):
        self.functions.CommandVel(1,0.00)
        print ("PIP joint stopped")

    def stopDIP(self):
        self.functions.CommandVel(3,0.00)
        print ("DIP joint stopped")

# Reset the scales    
    def reset (self):

        process = "not_done"
        while (process != "done"):
            if (Dip_angle>7):
                self.functions.CommandVel(1,-2)
                print("0")
            else : 
                self.functions.CommandVel(1,0)
                print("1")

            if(Pip_angle>7):
                self.functions.CommandVel(2,-2)
                print("3")
            else : 
                self.functions.CommandVel(2,0)
                print("4")

            if (MCPjointValue- first_value>7):
                self.functions.CommandVel(3,-2)
                print("5")
            else : 
                self.functions.CommandVel(3,0)
                print("6")

            if (Dip_angle<7 and Pip_angle<7 and (MCPjointValue- first_value)<7):
                process = "done"
                print("Joints are reset")

# Close the window
    def close_windows(self):
        self.technical.destroy()

'''--------------------------------- Settings page --------------------------------'''
class SettingsPage:
    def __init__(self, settings):
        
        self.settings = settings
        self.frame = tk.Frame(self.settings)
        self.settings.geometry ("700x800+400+50")
        self.settings.title("Settings")

    # Left Frame
        self.LeftFrameSettings = tk.Frame(settings)
        self.LeftFrameSettings.place(relx=0.017, rely=0.028, relheight=0.862
                                     , relwidth=0.47)
        self.LeftFrameSettings.configure(relief='groove')
        self.LeftFrameSettings.configure(borderwidth="2")
        self.LeftFrameSettings.configure(background="#ffffff")
        self.LeftFrameSettings.configure(width=285)

        self.LimiteAngleLabel = tk.Label(self.LeftFrameSettings, text="Limite angles", font='Helvetica 16 bold')
        self.LimiteAngleLabel.place(relx=0.316, rely=0.017, height=31, width=150)
        self.LimiteAngleLabel.configure(background="#ffffff")

        self.InDegreeLabel = tk.Label(self.LeftFrameSettings, text="(in degrees)", font='Helvetica 14')
        self.InDegreeLabel.place(relx=0.316, rely=0.06, height=31, width=150)
        self.InDegreeLabel.configure(background="#ffffff")

        self.MCPLabelSettings_ = tk.Label(self.LeftFrameSettings, text="MCP", font='Helvetica 14')
        self.MCPLabelSettings_.place(relx=0.07, rely=0.165, height=31, width=43)
        self.MCPLabelSettings_.configure(background="#ffffff")

        self.MCPScale = tk.Scale(self.LeftFrameSettings, from_=0.0, to=100.0)
        self.MCPScale.place(relx=0.281, rely=0.149, relwidth=0.639, relheight=0.0
                            , height=52, bordermode='ignore')
        self.MCPScale.configure(activebackground="#ececec")
        self.MCPScale.configure(background="#d9d9d9")
        self.MCPScale.configure(orient="horizontal")
        
        self.PIPLabelSettings = tk.Label(self.LeftFrameSettings, text="PIP", font='Helvetica 14')
        self.PIPLabelSettings.place(relx=0.07, rely=0.285, height=31, width=31)
        self.PIPLabelSettings.configure(background="#ffffff")

        self.PIPScale = tk.Scale(self.LeftFrameSettings, from_=0.0, to=100.0)
        self.PIPScale.place(relx=0.281, rely=0.269, relwidth=0.639, relheight=0.0
                            , height=52, bordermode='ignore')
        self.PIPScale.configure(background="#d9d9d9")
        self.PIPScale.configure(orient="horizontal")

        self.DIPLabelSettings = tk.Label(self.LeftFrameSettings, text="DIP", font='Helvetica 14')
        self.DIPLabelSettings.place(relx=0.07, rely=0.405, height=31, width=34)
        self.DIPLabelSettings.configure(background="#ffffff")

        self.DIPScale = tk.Scale(self.LeftFrameSettings, from_=0.0, to=100.0)
        self.DIPScale.place(relx=0.281, rely=0.389, relwidth=0.639, relheight=0.0
                            , height=52, bordermode='ignore')
        self.DIPScale.configure(background="#d9d9d9")
        self.DIPScale.configure(orient="horizontal")

        # Sensor  value threshold
        self.SensorThresholdLabel = tk.Label(self.LeftFrameSettings, text="Sensor Threshold", font='Helvetica 16 bold')
        self.SensorThresholdLabel.place(relx=0.15, rely=0.55, height=31, width=250)
        self.SensorThresholdLabel.configure(background="#ffffff")

        self.TopSensorLabel = tk.Label(self.LeftFrameSettings, text="Top", font='Helvetica 14')
        self.TopSensorLabel.place(relx=0.07, rely=0.73, height=31, width=34)
        self.TopSensorLabel.configure(background="#ffffff")
        
        self.TopSensorScale = tk.Scale(self.LeftFrameSettings, from_=0.0, to=1000.0)
        self.TopSensorScale.place(relx=0.281, rely=0.715, relwidth=0.639, relheight=0.0
                            , height=52, bordermode='ignore')
        self.TopSensorScale.configure(background="#d9d9d9")
        self.TopSensorScale.configure(orient="horizontal")
        self.TopSensorScale.set(top_sensor_threshold)

        self.BottomSensorLabel = tk.Label(self.LeftFrameSettings, text="Bottom", font='Helvetica 14')
        self.BottomSensorLabel.place(relx=0.03, rely=0.85, height=31, width=80)
        self.BottomSensorLabel.configure(background="#ffffff")
        
        self.BottomSensorScale = tk.Scale(self.LeftFrameSettings, from_=0.0, to=1000.0)
        self.BottomSensorScale.place(relx=0.281, rely=0.835, relwidth=0.639, relheight=0.0
                            , height=52, bordermode='ignore')
        self.BottomSensorScale.configure(background="#d9d9d9")
        self.BottomSensorScale.configure(orient="horizontal")
        self.BottomSensorScale.set(bottom_sensor_threshold)

    # Right Frame
        self.RightFrameSettings = tk.Frame(settings)
        self.RightFrameSettings.place(relx=0.495, rely=0.028, relheight=0.862
                                      , relwidth=0.487)
        self.RightFrameSettings.configure(relief='groove')
        self.RightFrameSettings.configure(borderwidth="2")
        self.RightFrameSettings.configure(background="#ffffff")
        self.RightFrameSettings.configure(width=295)

        # Labels
        self.VelocityLabel = tk.Label(self.RightFrameSettings, text="Velocities (in %)", font='Helvetica 16 bold')
        self.VelocityLabel.place(relx=0.3, rely=0.017, height=31, width=200)
        self.VelocityLabel.configure(background="#ffffff")

        self.V1LabelSettings = tk.Label(self.RightFrameSettings, text="V1", font='Helvetica 12')
        self.V1LabelSettings.place(relx=0.034, rely=0.198, height=31, width=27)
        self.V1LabelSettings.configure(background="#1e8bc3")
        self.V1LabelSettings.configure(foreground=white_color) 

        self.V2LabelSettings = tk.Label(self.RightFrameSettings,text="V2", font='Helvetica 12')
        self.V2LabelSettings.place(relx=0.034, rely=0.496, height=31, width=27)
        self.V2LabelSettings.configure(background="#1e8bc3")
        self.V2LabelSettings.configure(foreground=white_color) 

        self.V3LabelSettings = tk.Label(self.RightFrameSettings,text="V3", font='Helvetica 12')
        self.V3LabelSettings.place(relx=0.034, rely=0.793, height=31, width=27)
        self.V3LabelSettings.configure(background="#1e8bc3")
        self.V3LabelSettings.configure(foreground=white_color) 

        self.MCP1Label = tk.Label(self.RightFrameSettings, text="MCP", font='Helvetica 12')
        self.MCP1Label.place(relx=0.169, rely=0.116, height=31, width=43)
        self.MCP1Label.configure(background="#ffffff")

        self.PIP1Label = tk.Label(self.RightFrameSettings, text="PIP", font='Helvetica 12')
        self.PIP1Label.place(relx=0.203, rely=0.198, height=31, width=31)
        self.PIP1Label.configure(background="#ffffff")
        self.PIP1Label.configure(disabledforeground="#a3a3a3")
        self.PIP1Label.configure(foreground="#000000")
        self.PIP1Label.configure(text='''PIP''')

        self.DIP1Label = tk.Label(self.RightFrameSettings,text="DIP", font='Helvetica 12')
        self.DIP1Label.place(relx=0.203, rely=0.281, height=31, width=34)
        self.DIP1Label.configure(background="#ffffff")

        self.MCP2Label = tk.Label(self.RightFrameSettings, text="MCP", font='Helvetica 12')
        self.MCP2Label.place(relx=0.169, rely=0.413, height=31, width=43)
        self.MCP2Label.configure(background="#ffffff")

        self.PIP2Label = tk.Label(self.RightFrameSettings,text="PIP", font='Helvetica 12')
        self.PIP2Label.place(relx=0.203, rely=0.496, height=31, width=31)
        self.PIP2Label.configure(background="#ffffff")

        self.DIP2Label = tk.Label(self.RightFrameSettings,text="DIP", font='Helvetica 12')
        self.DIP2Label.place(relx=0.203, rely=0.579, height=31, width=34)
        self.DIP2Label.configure(background="#ffffff")

        self.MCP3Label = tk.Label(self.RightFrameSettings, text="MCP", font='Helvetica 12')
        self.MCP3Label.place(relx=0.169, rely=0.711, height=31, width=43)
        self.MCP3Label.configure(background="#ffffff")

        self.PIP3Label = tk.Label(self.RightFrameSettings,text="PIP", font='Helvetica 12')
        self.PIP3Label.place(relx=0.203, rely=0.793, height=31, width=31)
        self.PIP3Label.configure(background="#ffffff")

        self.DIP3Label = tk.Label(self.RightFrameSettings, text="DIP", font='Helvetica 12')
        self.DIP3Label.place(relx=0.203, rely=0.86, height=31, width=34)
        self.DIP3Label.configure(background="#ffffff")


        # Scale widgets
        self.V1ScalePIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V1ScalePIP.place(relx=0.339, rely=0.188, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V1ScalePIP.configure(background="#d9d9d9")
        self.V1ScalePIP.configure(orient="horizontal")

        self.V1ScaleDIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V1ScaleDIP.place(relx=0.339, rely=0.263, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V1ScaleDIP.configure(background="#d9d9d9")
        self.V1ScaleDIP.configure(orient="horizontal")

        self.V1ScaleMCP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V1ScaleMCP.place(relx=0.339, rely=0.099, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V1ScaleMCP.configure(background="#d9d9d9")
        self.V1ScaleMCP.configure(orient="horizontal")

        self.V2ScalePIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V2ScalePIP.place(relx=0.339, rely=0.485, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V2ScalePIP.configure(background="#d9d9d9")
        self.V2ScalePIP.configure(orient="horizontal")

        self.V2ScaleDIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V2ScaleDIP.place(relx=0.339, rely=0.562, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V2ScaleDIP.configure(background="#d9d9d9")
        self.V2ScaleDIP.configure(orient="horizontal")

        self.V2ScaleMCP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V2ScaleMCP.place(relx=0.339, rely=0.397, relwidth=0.603, relheight=0.0, height=52, bordermode='ignore')
        self.V2ScaleMCP.configure(background="#d9d9d9")
        self.V2ScaleMCP.configure(orient="horizontal")

        self.V3ScalePIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V3ScalePIP.place(relx=0.339, rely=0.783, relwidth=0.617, relheight=0.0, height=52, bordermode='ignore')
        self.V3ScalePIP.configure(background="#d9d9d9")
        self.V3ScalePIP.configure(orient="horizontal")

        self.V3ScaleDIP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V3ScaleDIP.place(relx=0.339, rely=0.86, relwidth=0.617, relheight=0.0, height=52, bordermode='ignore')
        self.V3ScaleDIP.configure(background="#d9d9d9")
        self.V3ScaleDIP.configure(orient="horizontal")

        self.V3ScaleMCP = tk.Scale(self.RightFrameSettings, from_=0.0, to=100.0)
        self.V3ScaleMCP.place(relx=0.339, rely=0.694, relwidth=0.617, relheight=0.0, height=52, bordermode='ignore')
        self.V3ScaleMCP.configure(background="#d9d9d9")
        self.V3ScaleMCP.configure(orient="horizontal")

    # Bottom Frame
        self.BottomFrame = tk.Frame(settings)
        self.BottomFrame.place(relx=0.017, rely=0.897, relheight=0.089, relwidth=0.965)
        self.BottomFrame.configure(relief='groove')
        self.BottomFrame.configure(borderwidth="2")
        self.BottomFrame.configure(background=white_color)

        self.SaveSettingsButton = tk.Button(self.BottomFrame, text="Save", font='Helvetica 12', command=self.Save)
        self.SaveSettingsButton.place(relx=0.427, rely=0.182, height=45, width=100)
        self.SaveSettingsButton.configure(background="#45d87b")
        self.SaveSettingsButton.configure(foreground=white_color)

        self.QuitSettingsButton = tk.Button(self.BottomFrame, text="Quit", command=self.go_back, font='Helvetica 12')
        self.QuitSettingsButton.place(relx=0.638, rely=0.182, height=45, width=100)
        self.QuitSettingsButton.configure(background="#666666")
        self.QuitSettingsButton.configure(foreground=white_color)

        self.ResetSettingsButton = tk.Button(self.BottomFrame, text="Reset", font='Helvetica 12', command=self.reset)
        self.ResetSettingsButton.place(relx=0.216, rely=0.182, height=45, width=100)
        self.ResetSettingsButton.configure(background="#f15a22")
        self.ResetSettingsButton.configure(foreground=white_color)

#--------------------------- Functions Settings page ----------------------------

#Reset the values
    def reset (self):
        self.MCPScale.set(0.0)
        self.PIPScale.set(0.0)
        self.DIPScale.set(0.0)
        
        self.V1ScaleMCP.set(0.0)
        self.V1ScalePIP.set(0.0)
        self.V1ScaleDIP.set(0.0)
        
        self.V2ScaleMCP.set(0.0)
        self.V2ScalePIP.set(0.0)
        self.V2ScaleDIP.set(0.0)
        
        self.V3ScaleMCP.set(0.0)
        self.V3ScalePIP.set(0.0)
        self.V3ScaleDIP.set(0.0)
        
        self.TopSensorScale.set(0.0)
        self.BottomSensorScale.set(0.0)

#Save the values        
    def Save (self):
                
        global V1_MCP_value
        global V1_PIP_value
        global V1_PIP_value

        global V2_MCP_value
        global V2_PIP_value
        global V2_DIP_value

        global V3_MCP_value
        global V3_PIP_value
        global V3_DIP_value

        global top_sensor_threshold
        global bottom_sensor_threshold

        MCP_Angle_value = self.MCPScale.get()
        print("MCP Angle :" ,MCP_Angle_value)
        PIP_Angle_value = self.PIPScale.get()
        print("PIP Angle :" ,PIP_Angle_value)
        DIP_Angle_value = self.DIPScale.get()
        print("DIP Angle :" ,DIP_Angle_value)
        
        V1_MCP_value = self.V1ScaleMCP.get()
        print("MCP velocity for v1 :" ,V1_MCP_value)
        V1_PIP_value = self.V1ScalePIP.get()
        print("PIP velocity for v1 :" ,V1_PIP_value)
        V1_PIP_value = self.V1ScaleDIP.get()
        print("DIP velocity for v1 :" ,V1_PIP_value)
        
        V2_MCP_value = self.V2ScaleMCP.get()
        print("PIP velocity for v2 :" ,V2_MCP_value)
        V2_PIP_value = self.V2ScalePIP.get()
        print("PIP velocity for v2 :" ,V2_PIP_value)
        V2_DIP_value = self.V2ScaleDIP.get()
        print("DIP velocity for v2 :" ,V2_DIP_value)
        
        V3_MCP_value = self.V3ScaleMCP.get()
        print("MCP velocity for v3 :" ,V3_MCP_value)
        V3_PIP_value = self.V3ScalePIP.get()
        print("PIP velocity for v3 :" ,V3_PIP_value)
        V3_DIP_value = self.V3ScaleDIP.get()
        print("DIP velocity for v3 :" ,V3_DIP_value)

        top_sensor_threshold = self.TopSensorScale.get()
        self.TopSensorScale.set(top_sensor_threshold)
        print("Top value threshold is :" ,top_sensor_threshold)
        bottom_sensor_threshold = self.BottomSensorScale.get()
        print("Bottom value threshold is :" ,bottom_sensor_threshold)

#Quit the window        
    def go_back(self):
        self.settings.destroy()

'''----------------------- 2 Callbacks for ROS subsriber --------------------------'''
# This funtion is called everytime "/Sensor_value" receives a value
def callback(data):
    global BottomValue
    global TopValue 
    global Pip_angle
    global Dip_angle
    global Pip_angle_array
    global Dip_angle_array
    global top_sensor_array
    global bottom_sensor_array
    global real_value
    global now

    # Separate the paquet received in a try catch to avoid unwanted data format
    try:
        sensorValue= data.data
        BottomValue= int(sensorValue[:3])
        TopValue = int(sensorValue[5:8])
        BottomValueNewton = BottomValue/125
        TopValueNewton = TopValue/125
        Pip_angle = int(float(sensorValue[10:13]))
        #print("pip",Pip_angle)
        Dip_angle = int(float(sensorValue[15:]))
        #print ("---------dip",Dip_angle)

    except ValueError:
        print ("Oops!  It seems that they was an invalid number for the PIP, DIP or sensors values.  Try again...")
        messagebox.showwarning("Error", " It seems that they was an invalid number for the PIP, DIP or sensors values ")

#Create a big matrix to stored every sensors values
    top_sensor_array.append(BottomValueNewton)
    #print(BottomValue)
    bottom_sensor_array.append(TopValueNewton)
    #print("--------------------",TopValue)
    Pip_angle_array.append(Pip_angle)
    Dip_angle_array.append(Dip_angle)
#Store the time at the same index for the plots
    real_value = (time.time() - first_time)
    now.append(real_value)

# This funtion is called everytime "/Flexor_controller/state" receives a value
def callback2(data):
    global MCPjointValue
    global MCPjointValue_array
    global TimeJointValues_array
    global timejoint
    global checkFirstValue 
    global first_value

    if (checkFirstValue == 0) :
        first_value = math.degrees(data.current_pos)
        checkFirstValue = 1
    MCPjointValueTemp = int((math.degrees(data.current_pos) - first_value))
    if MCPjointValueTemp < 120 :
        MCPjointValue = MCPjointValueTemp
        timejoint = (time.time() - first_time)
        TimeJointValues_array.append(timejoint)
        #print("----------------------MCP",MCPjointValue)
        MCPjointValue_array.append(MCPjointValue)

'''------------------------------- Main part -------------------------------'''
# Main which loops indefinitely 
def main():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/Sensor_value", String, callback)
    rate = rospy.Rate(20) # 10hz
    sub = rospy.Subscriber("/Flexor_controller/state", JointStateDynamixel, callback2) 
    rate.sleep()
    root = tk.Tk()    
    app = MainPage(root)
    root.mainloop()

if __name__ == '__main__':
    main()
