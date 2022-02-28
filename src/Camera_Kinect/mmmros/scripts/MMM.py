# Load Libraries
import serial

# The Multipurpose Mobile Manipulator Class
class MMM:
  # Parsing variables
  dataStr = "";
  dataIndex = 0;
  data = [ 0,0 ]; #[ Left UF, Right UF, Yaw]
  
  # Resets all values
  def reset(self):
    #Wheels (-255 to 255)
    self.leftWheel = 0
    self.rightWheel = 0
    #Shoulders (0-180)
    self.leftShoulder = 90
    self.rightShoulder = 90
    #Elbows (0-180), negative value turns servo off
    self.leftElbow = 90
    self.rightElbow =90
    #Arms (0-6000)
    self.leftArm = 0
    self.rightArm = 0
    #Left Grippers (0-180)
    self.L1 = 90
    self.L2 = 90
    self.L3 = 90
    self.L4 = 90
    self.L5 = 90
    #Right Grippers (0-180)
    self.R1 = 90
    self.R2 = 90
    self.R3 = 90
    self.R4 = 90
    self.R5 = 90
    
  # helper function clamps a value to a min/max
  def clamp(self, value, minimum, maximum):
    return min(max(value, minimum), maximum)
  # helper function clamps all robot values
  def clampAll(self):
    # Clamp Wheels
    self.leftWheel = self.clamp(self.leftWheel,-255,255)
    self.rightWheel = self.clamp(self.rightWheel,-255,255)
    # Clamp Shoulders
    self.leftShoulder = self.clamp(self.leftShoulder,0,180)
    self.rightShoulder = self.clamp(self.rightShoulder,0,180)
    # Clamp Elbows
    self.leftElbow = self.clamp(self.leftElbow,0,180)
    self.rightElbow = self.clamp(self.rightElbow,0,180)
    # Clamp Arms
    self.leftArm = self.clamp(self.leftArm,0,6000)
    self.rightArm = self.clamp(self.rightArm,0,6000)
    # Clamp Left Grippers 
    self.L1 = self.clamp(self.L1,0,180)
    self.L2 = self.clamp(self.L2,0,180)
    self.L3 = self.clamp(self.L3,0,180)
    self.L4 = self.clamp(self.L4,0,180)
    self.L5 = self.clamp(self.L5,0,180)
    # Clamp Right Grippers 
    self.R1 = self.clamp(self.R1,0,180)
    self.R2 = self.clamp(self.R2,0,180)
    self.R3 = self.clamp(self.R3,0,180)
    self.R4 = self.clamp(self.R4,0,180)
    self.R5 = self.clamp(self.R5,0,180)
  # helper function maps one range of values to another
  def translate(self, value, inMin, inMax, outMin, outMax):
    # Clamp value within input range
    value = self.clamp(value, inMin, inMax)
    # Figure out how 'wide' each range is
    inSpan = inMax - inMin
    outSpan = outMax - outMin
    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - inMin) / float(inSpan)
    # Convert the 0-1 range into a value in the right range.
    return outMin + (valueScaled * outSpan) 
    
  # spin motors in meters/sec (-.18 to .18 m/s)
  def setWheelVelocity(self, leftSpeed, rightSpeed):
    #convert speed to robot units
    self.leftWheel = int(self.translate(leftSpeed, -.18,.18, -255.0,255.0))
    self.rightWheel = int(self.translate(rightSpeed, -.18,.18, -255.0,255.0))   
  # rotate shoulders in degrees (0 to 135 degrees)
  def rotateShoulders(self, leftAngle, rightAngle):
    #convert angles to robot units
    self.leftShoulder = int(self.translate(leftAngle, 0,120, 0,180))
    self.rightShoulder = int(self.translate(rightAngle, 0,120, 0,180)) 
  # rotate elbows in degrees (-60 to 60 degrees)
  def rotateElbows(self, leftAngle, rightAngle):
    #convert angles to robot units
    self.leftElbow = int(self.translate(leftAngle, -60,60, 0,180))
    self.rightElbow = int(self.translate(rightAngle, -60,60, 0,180)) 
  # extend arms in meters (0 to .127 m)
  def extendArms(self, leftAmount, rightAmount):
    #convert meters to robot units
    self.leftArm = int(self.translate(leftAmount, 0,.127, 0,6000.0))
    self.rightArm = int(self.translate(rightAmount, 0,.127, 0,6000.0))
  # control left grippers
  def setLeftGrippers(self, l1 = 0,l2 = 0,l3 = 0,l4 = 0,l5 = 0):
    self.L1 = l1;
    self.L2 = l2;
    self.L3 = l3;
    self.L4 = l4;
    self.L5 = l5;
  # control right grippers
  def setRightGrippers(self,r1 = 0,r2 = 0,r3 = 0,r4 = 0,r5 = 0):
    self.R1 = r1;
    self.R2 = r2;
    self.R3 = r3;
    self.R4 = r4;
    self.R5 = r5;
  
  # Get Left RangeFinder Data
  def getLeftRange(self):
    return self.data[0]
  
  # Get Right RangeFinder Data
  def getRightRange(self):
    return self.data[1]
  
  # Reads data coming from Arduino
  def parseData(self):
    #Make backup in case parsing fails.
    oldData = list(self.data)
    
    #Read data
    try:
      for chr in self.ser.readline():   
        #begin parsing
        if(chr == '{'):
          self.dataStr = ""
          self.dataIndex = 0 
        #extract number from data
        elif(chr == ',' or chr == '}'):
          self.data[self.dataIndex] = int(self.dataStr);
          self.dataIndex+=1; self.dataStr = "";
          #Finished reading message
          if(chr == '}'): 
            self.dataStr = "" 
            self.dataIndex = 0
        #add char to current string
        elif (chr.isspace() == False and chr != '\n'):
          self.dataStr += chr;
    except:
      self.data = oldData
    self.ser.flushInput();
    
  # Sends data to Arduino in Robot units, which updates position 
  def update(self):
    # Prepare data
             #Wheels
    data = [ self.leftWheel, self.rightWheel,
             #Shoulders
             self.leftShoulder, self.rightShoulder, 
             #Elbows
             self.leftElbow, self.rightElbow,
             #Arms
             self.leftArm, self.rightArm,
             #Left Grippers
             self.L1, self.L2, self.L3, self.L4, self.L5,
             #Right Grippers
             self.R1, self.R2, self.R3, self.R4, self.R5 ]
                             
    # begin dataString
    dataString = "{ "
    # Add values to data list
    for dataIndex in range(len(data)):
      # Add data as a string
      dataString += str(int(data[dataIndex]))
      if (dataIndex < len(data)-1):
        dataString += ", "; # to next value
      else: 
        dataString += " }" # finish string
      
    # send data to Arduino
    try:
      self.ser.flush()
      self.ser.flushInput()
      self.ser.write(dataString)
      sensors = " : [ " + str(self.data[0]) + ", " + str(self.data[1]) + " ]"
      #print dataString + sensors + "\n"  
    except:
      print("Could not update robot at " + self.ser.name + "!")
      print("(Check USB connection to Arduino Mega)")
    
  # Initializes the MMM class  
  def __init__(self, portName):
    try:
      self.ser = serial.Serial(portName,57600,timeout=0)
    except:
      print("Could not connect to robot at " + portName + "!")
      print("(Check USB connection to Arduino Mega)")
      quit()
    
    self.ser.readline()
    self.reset()
    self.update()
    