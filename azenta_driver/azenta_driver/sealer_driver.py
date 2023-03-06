from ast import Pass
import os.path
import time
import serial
import logging
import re

#Log Configuration
# file_path = os.path.join(os.path.split(os.path.dirname(__file__))[0]  + '/sealer_logs/sealer_logs.log')
# logging.basicConfig(filename = file_path, level=logging.DEBUG, format = '[%(levelname)s] [%(asctime)s] [%(name)s] %(message)s', datefmt = '%Y-%m-%d %H:%M:%S')


class A4S_SEALER_DRIVER():
    """
    Description: 
                 - Python interface that allows remote commands to be executed using simple string messages over TCP/IP on PF400 cobot. 
    Serial Communication Messages from the Robot:
                 - Responses begin with a "0" if the command was successful, or a negative error code number
    """
    def __init__(self, host_path, baud_rate=19200):
        '''
        This function initializes the data to be called and modified in other locations in the client.
        '''

        self.host_path = host_path
        self.baud_rate = baud_rate
        self.connection = None
        self.connect_sealer()
        self.output=''
        self.status = ''
        self.heat=''

    def connect_sealer(self):
        '''
        Connect to serial port / If wrong port entered inform user 
        '''

        #Connect to serial port / If wrong port entered inform user 
        try:
            self.connection = serial.Serial(self.host_path, self.baud_rate)
        except serial.SerialException as connection_error:
            print(connection_error)     

    def response_fun(self, time_wait):                         
        '''
        Records the data outputted by the Peeler and sets it to equal "" if no data is outputted in the provided time.
        '''

        response_timer = time.time()
        while time.time() - response_timer < time_wait: 
            if self.connection.in_waiting != 0:           
                response = self.connection.read_until(expected=b'!')
                response_string = response.decode('utf-8')
                response_string_pat = re.search(r"=\d+,(\d+),(\d+),\d+,\d+,\d+", response_string)
                if response_string_pat:
                    self.status=int(response_string_pat[1])
                    self.heat=int(response_string_pat[2])
                    print('status = ' + str(self.status))
                break
            else:
                response_string = ""
        return response_string


    def send_command(self, command, success_msg="", err_msg="", timeout=10):
        ''' 
        '''

        self.output = self.output+ 'Command: ' + command + "\n"
        
        self.connection.write(command.encode('utf-8'))        

        ready_timer = time.time()
        response_buffer = ""

        while self.status!=0:
            new_string = self.response_fun(timeout)
            print(new_string)
            if new_string != "":
                self.output = self.output + new_string + '\n'

            response_buffer = response_buffer + new_string
            
            if time.time() - ready_timer > 20:
                print('timed out')
                break

        return response_buffer

    def get_status(self,timeout=500):
        self.response_fun(timeout)     

    def get_error(self):
        pass

    def reset(self):
        '''
        Clears error status/resets shuttle.
        '''

        cmd_string = '*00SR=zz!'
        success_msg = "Conducting System Reset"
        err_msg = "Failed to Conduct System Reset"
        self.send_command(cmd_string, success_msg, err_msg)

    def open_gate(self):
        '''
        Opens shuttle
        '''

        cmd_string = '*00MO=zz!'
        success_msg = "Opening Gate"
        err_msg = "Failed to Open Gate"
        self.send_command(cmd_string, success_msg, err_msg)

    def close_gate(self):
        '''
        Closes shuttle
        '''
        
        cmd_string = '*00MC=zz!'
        success_msg = "Closing Gate"
        err_msg = "Failed to Close Gate"          
        self.send_command(cmd_string, success_msg, err_msg)

    def set_temp(self, temp=175):
        '''
        Adjusts seal to given temperature.
        '''
        temp = str(temp).zfill(4)
        cmd_string = f'*00DH={temp}zz!'
        success_msg = "Setting Temp. to %d°C"%(temp)
        err_msg = "Failed to Set Temp. to %d°"%(temp)         
        self.send_command(cmd_string, success_msg, err_msg)
       
    def set_time(self, time=3.0):
        '''
        Adjusts seal time to given time.
        '''
        time = str(int(time*10)).zfill(4)
        cmd_string = f'*00DT={time}zz!'
        success_msg = "Setting Seal Time to %s S"%(time)
        err_msg = "Failed to Set Seal Time to %s S"%(time)
        self.send_command(cmd_string, success_msg, err_msg)

    def seal(self):
        '''
        Conducts seal action.
        '''

        cmd_string = '*00GS=zz!'
        success_msg = "Sealing"
        err_msg = "Failed to Seal"
        self.send_command(cmd_string, success_msg, err_msg)


    def config_robot(self, temp,time):
        '''
        Sets robot to given permission/time
        '''

        self.set_temp(temp)
        self.set_time()



if __name__ == "__main__":
    '''
    Runs given function.
    '''

    sealer = A4S_SEALER_DRIVER("/dev/ttyUSB0")
    sealer.get_status()
    sealer.reset()
    sealer.get_status()
    sealer.get_error()
    # dummy_seal.reset()
    # dummy_seal.reset()
    
