# -*- coding: utf-8 -*-
"""
Created on Mon Dec 21 12:10:07 2020

@author: whatif
"""


# -*- coding: utf-8 -*-
"""
Created on Sun Dec 20 23:35:44 2020

@author: whatif
"""


import pyqtgraph as pg
import array
import serial
import threading
import numpy as np
import time
import PySimpleGUI as sg


filetowrite= open("C:/Users/whatif/Desktop/cart_plot_info.csv","w")
fileline = 0

system_status_flag=1
system_send_serial_flag=0
system_send_serial_string_content=''

print_data_type=0

i = 0

def Serial():
    while(system_status_flag):
        dat=mSerial.readline()
        if(dat):
            if data!=" ":                
                
                print(dat)
                
              
                dat_decode=dat.decode()
                
                infotypebit=dat_decode[0]
                dat1=dat_decode[1:]
                
                if (infotypebit=='i'):
                    print(dat1)
                
                if (infotypebit=='d'):
                    
                    data_info=dat1.split(" ")
                    
                    # interesting , some obviously code doesnt work well here, I need to use a Kx as inttermediate variable to hanle any conversion
                    k0=data_info[0]
                    k1=data_info[1]
                    k2=data_info[2]
                    k3=data_info[3]
                    
                    global fileline
                    
                    fileline = fileline +1;
                    filetowrite.write(str(fileline)+","+k0+","+k1+","+k2+","+k3+"\n") 
                
                                       
                    '''
                    global i;
                    if i < historyLength:
                        data[i] = float(k0)
                        data1[i] = float(k1)
                        data2[i] = float(k2)
                        data3[i] = float(k3)
                        i = i+1
                    else:
                        data[:-1] = data[1:]
                        data[i-1] = float(k0)
                        
                        data1[:-1] = data1[1:]
                        data1[i-1] = float(k1)
                        
                        data2[:-1] = data2[1:]
                        data2[i-1] = float(k2)
                        
                        data3[:-1] = data3[1:]
                        data3[i-1] = float(k3)
                    ''' 
    

    print('Serial Connection Thread is closed\n')
            
                
def plotData():
    
    curve.setData(data)    
    curve1.setData(data1)  
    curve2.setData(data2)  
    curve3.setData(data3)
    
    if (not system_status_flag):
        timer.stop() 
        app.quit()
        mSerial.close() 
        print('time stopped and waveform windows app quit')
        

def start_app():
    app.exec_()
    

def command_window():
    global system_status_flag
    
 
    # Define the window's contents
    layout = [[sg.Text("Key in your cart command")],
              [sg.Input(size=(40,1),key='-INPUT-')],
              [sg.Text('',size=(40,1),key='-OUTPUT-')],              
              [sg.Button('Start'), sg.Button('Set_PI'),sg.Button('Motor_on'),sg.Button('Motor_off'), sg.Button('Plot'),sg.Button('Send'), sg.Button('Cancle'),sg.Button('Quit_Sys')], 
              [sg.Button('prt_angle_acc_set_meas'), sg.Button('prt_ref_pid_set_msd'),sg.Button('prt_pid_acc_set_msd'),sg.Button('prt_pid_acc_ang_msd')],
              [sg.Button('open_file'), sg.Button('close_file')],
              
              [sg.Text("-------- Parameter I SETTING : SPEED2PWMDUTY GAIN Ks2d and Ks2d_offset--------")],
              
              [sg.Text("Ks2d:"),sg.Text('',size=(6,1),key='-OUTPUTK1-'),
               sg.Text("Ks2d_offset:"),sg.Text('',size=(6,1),key='-OUTPUTK2-')],
              
              [sg.Slider((0, 1), 1, 0.01, orientation="h", size=(40, 7), key="-INPUTK1-",)],
              [sg.Slider((0, 1), 0, 0.01, orientation="h", size=(40, 7), key="-INPUTK2-",)],
                            
              [sg.Button('UPDATE para I')],
              
              
              
              
              [sg.Text("-------- Parameter II SETTING : PID loop Kp Ki Kd--------")],
              
              [sg.Text("Kp:"),sg.Text('',size=(6,1),key='-OUTPUTK4-'),
               sg.Text("Ki:"),sg.Text('',size=(6,1),key='-OUTPUTK5-'),
               sg.Text("Kd:"),sg.Text('',size=(6,1),key='-OUTPUTK6-')],
              
              [sg.Slider((0, 10), 0, 0.01, orientation="h", size=(40, 7), key="-INPUTK4-",)],
              [sg.Slider((0, 1), 0, 0.01, orientation="h", size=(40, 7), key="-INPUTK5-",)],
              [sg.Slider((0, 1), 0, 0.01, orientation="h", size=(40, 7), key="-INPUTK6-",)],
              
              [sg.Button('UPDATE para II')],
            
              [sg.Text("-------- Parameter III SETTING : Ref Speed Setting before Plant ---")],
              
              [sg.Text("Speed Reference:"),sg.Text('',size=(6,1),key='-OUTPUTK7-')],
              [sg.Slider((-1.5, 1.5), 0, 0.01, orientation="h", size=(40, 7), key="-INPUTK7-",)],
              
              [sg.Button('UPDATE para III')],
              
              
              [sg.Text("-------- Parameter IV SETTING : Acceleration gain --------")],
              
              [sg.Text("Kangle:"),sg.Text('',size=(6,1),key='-OUTPUTK8-'),
               sg.Text("Komega:"),sg.Text('',size=(6,1),key='-OUTPUTK9-')],
              
              [sg.Slider((0, 350), 190, 0.01, orientation="h", size=(40, 7), key="-INPUTK8-",)],
              [sg.Slider((0, 50), 14, 0.001, orientation="h", size=(40, 7), key="-INPUTK9-",)],
                            
              [sg.Button('UPDATE para IV')],

              [sg.Text("-------- Parameter V SETTING : angle_omega_threathold --------")],
              
              [sg.Text("angle_omega_threathold:"),sg.Text('',size=(6,1),key='-OUTPUTK10-')],
              
              [sg.Slider((0, 0.01), 0, 0.0001, orientation="h", size=(40, 7), key="-INPUTK10-",)],
                            
              [sg.Button('UPDATE para V')]
                
              
             ]

    # Create the window
    window = sg.Window('chenjian debug interface window', layout)

    
    # Display and interact with the Window using an Event Loop
    while True:
        event, values = window.read()
        # See if user wants to quit or window was closed
        if event == 'Cancle':            
            window['-INPUT-'].update('' )        
            window['-OUTPUT-'].update('' )
            
        elif event == 'Start':    
            mSerial.write("g".encode())    
            
        elif event == 'Set_PI':     
            mSerial.write("v".encode())       
            
        elif event == 'Motor_on':         
            mSerial.write("t".encode()) 
            
        elif event == 'Motor_off':   
            mSerial.write("s".encode()) 
            
        elif event == 'Plot':       
            mSerial.write("p".encode())      
            
        elif event == 'UPDATE para I':    
            window['-OUTPUTK1-'].update(values['-INPUTK1-'] )  
            window['-OUTPUTK2-'].update(values['-INPUTK2-'] ) 
            serial_write_dat="x1%"+str(values['-INPUTK1-'])+"%"+str(values['-INPUTK2-'])+"%0%"
            print(serial_write_dat)
            mSerial.write(serial_write_dat.encode())  
            
        elif event == 'UPDATE para II':    
            window['-OUTPUTK4-'].update(values['-INPUTK4-'] )  
            window['-OUTPUTK5-'].update(values['-INPUTK5-'] ) 
            window['-OUTPUTK6-'].update(values['-INPUTK6-'] )  
            serial_write_dat="x2%"+str(values['-INPUTK4-'])+"%"+str(values['-INPUTK5-'])+"%"+str(values['-INPUTK6-'])+"%"
            print(serial_write_dat)
            mSerial.write(serial_write_dat.encode())  
            
        elif event == 'UPDATE para III':    
            window['-OUTPUTK7-'].update(values['-INPUTK7-'] )  
            serial_write_dat="x3%"+str(values['-INPUTK7-'])+"%0%0%"
            print(serial_write_dat)
            mSerial.write(serial_write_dat.encode())  
            
        elif event == 'UPDATE para IV':    
            window['-OUTPUTK8-'].update(values['-INPUTK8-'] )  
            window['-OUTPUTK9-'].update(values['-INPUTK9-'] ) 
            serial_write_dat="x4%"+str(values['-INPUTK8-'])+"%"+str(values['-INPUTK9-'])+"%0%"
            print(serial_write_dat)
            mSerial.write(serial_write_dat.encode())  
            
        elif event == 'UPDATE para V':    
            window['-OUTPUTK10-'].update(values['-INPUTK10-'] )  
            serial_write_dat="x5%"+str(values['-INPUTK10-'])+"%0%0%"
            print(serial_write_dat)
            mSerial.write(serial_write_dat.encode())  
            
        elif  event == sg.WINDOW_CLOSED or event == 'Quit_Sys':
            break
        
        elif event == 'prt_angle_acc_set_meas':   
            mSerial.write('k0'.encode()) 
            
        elif event == 'prt_ref_pid_set_msd':  
            mSerial.write('k1'.encode()) 
            
        elif event == 'prt_pid_acc_set_msd':  
            mSerial.write('k2'.encode())
            
        elif event == 'prt_pid_acc_ang_msd':  
            mSerial.write('k3'.encode())             

        
        elif event == 'Send':
            mSerial.write(values['-INPUT-'].encode()) 
            window['-OUTPUT-'].update('Command Sent out: ' + values['-INPUT-'])
            window['-INPUT-'].update('' )  
    
    
    system_status_flag=0
    window.close()  
    
    system_status_flag=0
    print ('command window thread is closed')
    
    filetowrite.close()    
    print ('data file is closed')
       


    
if __name__ == "__main__":
    

    
    historyLength = 150 
    a = 0
    
    
    
    # for speed_ref, speed_measure, speed_set, encoder_speed test
    
    if (print_data_type==0):
        
        plot_title=["Angle",
               "Angle_omega",
               "Speed_acc",
               "Speed_set"]
        
        plot_yRange=[3, 3.3,
                     -1.8,1.8,
                     -3,3,
                     -3,3]
    
    if (print_data_type==1):
        
        plot_title=["Speed_ref (m/s)",
               "Speed_pid_output",
               "Speed_set",
               "Speed_measured"]
        
        plot_yRange=[-1.8,1.8,
                     -1.8,1.8,
                     -1.8,1.8,
                     -1.8,1.8]

    if (print_data_type==2):
        
        plot_title=["Speed_pid_output (m/s)",
               "Speed_acc (m/s)",
               "Speed_set",
               "Speed_measured"]
        
        plot_yRange=[-1.8,1.8,
                     -1.8,1.8,
                     -1.8,1.8,
                     -1.8,1.8]
    
    
    
    '''
    # for angle , omega and acceleration test
    plot_title=["angle",
           "omega (degree/s)",
           "Speed_acceleration",
           "Speed_measured"]
    
    plot_yRange=[2.15,4.15,
                 -1.8,1.8,
                 -15,15,
                 -1.8,1.8]
    
    '''
    
    
    
    
    
    
    plot_xlable="time ( xx ms )"
    
    data = array.array('i') 
    data1 = array.array('i')     
    data2 = array.array('i') 
    data3 = array.array('i') 
    
    data=np.zeros(historyLength).__array__('d')
    data1=np.zeros(historyLength).__array__('d')
    data2=np.zeros(historyLength).__array__('d')
    data3=np.zeros(historyLength).__array__('d')
    
    

    app = pg.mkQApp() 
    
    win = pg.GraphicsWindow()
    win.setWindowTitle(u'Chen Jian Invert Pendulum Car Mornitor')
    win.resize(1200, 900)

    p = win.addPlot()
    p.showGrid(x=True, y=True) 
    p.setRange(xRange=[0, historyLength], yRange=[plot_yRange[0], plot_yRange[1]], padding=0)
    p.setLabel(axis='left', text=plot_title[0]) 
    p.setLabel(axis='bottom', text=plot_xlable)
    p.setTitle(plot_title[0]) 
    curve = p.plot() 
    curve.setData(data)    

    
    p1 = win.addPlot() 
    p1.showGrid(x=True, y=True) 
    p1.setRange(xRange=[0, historyLength], yRange=[plot_yRange[2], plot_yRange[3]], padding=0)
    p1.setLabel(axis='left', text=plot_title[1]) 
    p1.setLabel(axis='bottom', text=plot_xlable)
    p1.setTitle(plot_title[1])
    curve1 = p1.plot() # 
    curve1.setData(data1)
    
    win.nextRow() 
    
    p2 = win.addPlot() 
    p2.showGrid(x=True, y=True) 
    p2.setRange(xRange=[0, historyLength], yRange=[plot_yRange[4], plot_yRange[5]], padding=0)
    p2.setLabel(axis='left', text=plot_title[2]) 
    p2.setLabel(axis='bottom', text=plot_xlable)
    p2.setTitle(plot_title[2])
    curve2 = p2.plot() # 
    curve2.setData(data2)
    
    p3 = win.addPlot() 
    p3.showGrid(x=True, y=True) 
    p3.setRange(xRange=[0, historyLength], yRange=[plot_yRange[6], plot_yRange[7]], padding=0)
    p3.setLabel(axis='left', text=plot_title[3]) 
    p3.setLabel(axis='bottom', text=plot_xlable)
    p3.setTitle(plot_title[3])
    curve3 = p3.plot() # 
    curve3.setData(data3)
    
    
    
    
    
    
    #portx = 'COM3'
    portx = 'COM6'
    bps = 115200
    
    
    mSerial = serial.Serial(portx, int(bps))

    if (mSerial.isOpen()):
        print("open success")
        mSerial.flushInput()
    else:
        print("open failed")
        serial.close() 
    
    

    th1 = threading.Thread(target=Serial)
    th1.start()
    
    th2 = threading.Thread(target=start_app)
    th2.start()
    
   
    timer = pg.QtCore.QTimer()
    timer.timeout.connect(plotData) 
    timer.start(10) 
    
    
    #app.exec_()
    
    # notice : GUI window function can not be put into a thread. 
    # coding conflict.
    
    command_window(); 
    
    print('main code complete')



