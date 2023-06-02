# Projects

## Table of Contents
* [Table of Contents](#TableOfContents)
* [Robot Arm Project](#Robot_Arm_Project)
* [PID Box](#PID_Box)
---

## Robot_Arm_Project

### Project Description

In this project, we made an articulated robot arm which can be controlled through a few potentiometers. It is designed so that different toolheads can be easily swapped in and out, making use of magnets, meaning it can be used to pick up things, draw with a pencil, grab things with a magnet, or just about anything else. Other toolheads can be easily designed in the future, and be used for all kinds of applications.

Materials used:
- 3 180° degree servos
- 3 potentiometers
- battery pack
- 4 magnets for the head, and 4 more for each attachment
- arduino uno
- 8 standoffs
- breadboard
- wires
- 4-40 bolts, nuts, and locknuts
- lasercutter and 3D printer

### Code and Wiring

```python
#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

int pin1 = 9;
int pin2 = 10;
int pin3 = 11;

int pot1 = A0;
int pot2 = A1;
int pot3 = A2;

double state1 = 90;
double state2 = 90;
double state3 = 90;

double buffer = 50;
int delayLength = 5;

void setup() {
  Serial.begin(9600);
  servo1.attach(pin1);
  servo2.attach(pin2);
  servo3.attach(pin3);
}

void loop() {
  //servo 1
  state1 = analogRead(pot1);
  state1 = map(state1, 0, 1023, 1000, 13500); //map() only returns int, so map to a multiple of 100 and divide to get decimals
  state1 /= 100;
  servo1.write(state1);
  Serial.print(state1);
  Serial.print("     ");
  //servo2
  state2 = analogRead(pot2);
  state2 = map(state2, 0, 1023, 1000, 13500);
  state2 /= 100;
  servo2.write(state2);
  Serial.print(state2);
  Serial.print("     ");
  //servo3
  state3 = analogRead(pot3);
  state3 = map(state3, 0, 1023, 2000, 13500);
  state3 /= 100;
  servo3.write(state3);
  Serial.println(state3);
}
```

<img width="409" alt="wiring" src="https://user-images.githubusercontent.com/113116262/225998745-cab0ba79-55f3-4e7e-80db-314c34f4646b.png">

_Wiring diagram_

### Evidence

[Onshape Link](https://cvilleschools.onshape.com/documents/d6a7e94291a496a5fc988fe2/w/c47db00d9ece1087e6936482/e/6bb8c6e7503eb20d835169f3)

<img width="359" alt="robotArmAssembly" src="https://user-images.githubusercontent.com/113116262/225793507-db353efb-4f28-4f6d-a59c-728306d643d8.png"><img width="362" alt="head" src="https://user-images.githubusercontent.com/113116262/225982749-e140e9e5-97b5-47bf-a474-08dd8dcdea74.png">

_The arm and linkages. The head part contains holes for magnets, so that different toolheads can be easily attached and removed._

<img width="389" alt="baseAssembly" src="https://user-images.githubusercontent.com/113116262/225793579-b0e7cbbe-f3dc-49ca-aeaa-d5d4a7e48327.png"><img width="455" alt="base2" src="https://user-images.githubusercontent.com/113116262/225793529-14874869-1f79-4186-b0c6-c1f6b2e33bd4.png">

_The base parts to hold the arm. One platform is elevated so that a servo can be attached underneath._

<img width="422" alt="commandModule" src="https://user-images.githubusercontent.com/113116262/225985234-31749064-a48e-41a0-a92f-6820d50aad6b.png">

_The seperate plate containing wiring and labelled potentiometer mounts._

![arm1](https://user-images.githubusercontent.com/113116262/225988512-003b75d8-d280-46ab-ab38-a7887f701523.JPG)![arm2](https://user-images.githubusercontent.com/113116262/225988475-c693f362-bce6-4139-813e-7b26e649581f.JPG)


_The fully assembled and functional arm._

https://user-images.githubusercontent.com/113116262/226353837-671ebcff-0872-4e96-8308-afa104fb123a.mov

_A video demonstration of the arm's functionality._

### Reflection

In the end, the project is fully functional, and actually moves very smoothly. It took a while to get the movement to be smooth, though, because the servos kept jittering back and forth. After a few different solutions, a lot of troubleshooting, and a complete transition from using CircuitPython and the Metro boards to using Arduino, we found a good solution. We simplified the code a bunch, and that combined with good use of the ```map()``` function resulted in just about zero jittering in all of the servos.

We didn't run into too many issues with CAD, though we did have to find a solution to our goal of being able to easily swap the toolheads. We ended up settling on using magnets, and implementing them was realitively straightforward. The main linkage of our arm is based off of [this design](https://grabcad.com/library/modular-electromagnetic-robot-arm-1), though we have either edited or replaced most of the parts to customize them for our needs, or to be more efficiently fabricated. This linkage provides a smooth control system for the arm with only two potentiometers, while allowing for a large range of motion and keeping the head level with the ground.

Overall, I think that this project was pretty successful. It moves well in 3 dimensions, can complete an assortment of tasks, and the head can be easily swapped. It would have been cool if the heads were automatically interchangeable and didn't require human interference, but the system we have is pretty seamless.

## PID_Box

### Project Description

In this project, we tried to make a simple spinning wheel adjust its own speed towards a setpoint using PID.

### Code and Wiring

```python
from adafruit_motor import motor
from lcd.lcd import LCD
from lcd.i2c_pcf8574_interface import I2CPCF8574Interface
from PID_CPY import PID
lcdPower = digitalio.DigitalInOut(board.D9)
lcdPower.direction = digitalio.Direction.INPUT
lcdPower.pull = digitalio.Pull.DOWN

# Keep the I2C protocol from running until the LCD has been turned on
# You need to flip the switch on the breadboard to do this.
while lcdPower.value is False:
    print("still sleeping")
    time.sleep(0.1)

# Time to start up the LCD!
time.sleep(1)
print(lcdPower.value)
print("running")

i2c = board.I2C()
lcd = LCD(I2CPCF8574Interface(i2c, 0x27), num_rows=2, num_cols=16)

encoder = rotaryio.IncrementalEncoder(board.D3, board.D2)
last_position = 0
button = digitalio.DigitalInOut(board.D4)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP

photoI = digitalio.DigitalInOut(board.D7)
photoI.direction = digitalio.Direction.INPUT
photoI.pull = digitalio.Pull.UP

motor = AnalogOut(board.A0)
pid = PID(100, 10, 50, setpoint = 100)
pid.output_limits = 23000, 50000
prevState = True
prevTime = 0

maxSpeed = 100000   # constant, in rpm
interval = 10   # number of rpms to change by
trueSpeed = 0

lastTime, Output, Setpoint, errSum, lastErr = 0, 0, 0, 0, 0
motorPower = 0

# Variables for counting interrupts and RPM
intTime =0
interrupts =0
RPM = 0
lastVal = False
previous_time = time.monotonic()
rpmCheckTime = 0.5
rpmCheckState = True
photoVal = False           
oldPhotoVal = False # Used to make sure we only count the first loop when interupt is broken
time1 =0
time2 =0
RPM = 0 

while True:
    # rotary encoder code
    position = encoder.position

    if last_position is None or position != last_position:
        print(position)

        if(position > last_position and Setpoint < maxSpeed):
            Setpoint = Setpoint + interval
        if(position < last_position and Setpoint > 0):
            Setpoint = Setpoint - interval

        lcd.set_cursor_pos(0,0)
        lcd.print(f"{Setpoint}")
        pid.setpoint = Setpoint 
    last_position = position

    photoVal = photoI.value 

    # RPM math and calculations
    if photoVal and (oldPhotoVal == False):
        oldPhotoVal = True
        interrupts += 1
        
        if interrupts % 2 == 0:
            time2= time.monotonic()
            RPM = 1.0/((time2-time1))*60
            print(f"{photoI.value} {interrupts} Rpm: {RPM} 1")
            time1 = time2
        elif interrupts % 2 == 1:    # first 
            time2 = time.monotonic()
            RPM = 1.0/((time2-time1))*60
            print(f"{photoI.value} {interrupts} Rpm: {RPM} 2")
            time1 = time2
    if not photoVal:
        oldPhotoVal  = False

    # PID math
    motorPower = int(pid(RPM))

 
    print(f"Motor Power: {motorPower}")
    motor.value = motorPower

    # LCD output code
    lcd.set_cursor_pos(0,0)
    lcd.print(f"Setpoint: {Setpoint}rpm")
    lcd.set_cursor_pos(1,0)
    lcd.print(f"Speed: {RPM}rpm")
    
    print(f"Speed: {RPM}rpm   Setpoint: {Setpoint}rpm")
```

![Capture](https://github.com/egarcia28/ENG3_Projects/assets/112961319/e7d222a4-b857-4db4-8cec-800cddeeae7e)
_Wiring diagram_

### Evidence

![image](https://github.com/jhelmke45/projects/assets/113116262/bb1f8f0d-5e8e-4dea-b077-9fb3ce952a4d)![image](https://github.com/jhelmke45/projects/assets/113116262/180ad6be-af71-4ff6-ad34-25a8d53ccf93)

_CAD renderings of the base plates, and then of the full assembly, complete with imported parts_

![IMG_2032](https://github.com/egarcia28/ENG3_Projects/assets/112961319/8a83e18d-ae34-4eb0-8e69-f5aa2e79beba)
![IMG_2033](https://github.com/egarcia28/ENG3_Projects/assets/112961319/944db096-0a03-4d94-93b7-d35d2a84806a)

_Our physical version of the box_
### Reflection
