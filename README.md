# Tecbot RobotCode 2022

Hello, world!

This is the repository for the 2022 FRC Season.
Read more about it [here](https://www.firstinspires.org/robotics/frc/game-and-season).

Anyways, here is a rough overview of the subsystems:


# TODO 
- Joe:
  - PID. Shooter + Turret + Feeder !!
  - plus simple proportional control.
- view LAMBOT and BOTBUSTERS modes
  - 3 modes. 1; not doing anything. 2; looking but have not found yet. 3; aimed & ready to shoot.

	- actually today:
		- go for it.create a lineal function for the shooter velocity. starting from 0 to 0.5 to 1,
		- for several minutes. maybe standard deviation and median 

## DriveTrain
The drivetrain is the most important subsystem. It
allows the robot to move on the floor using wheel and motors.
It physically consists of three wheel per side and two
motors (NEO) per side. Each motor is wired to a 
motor controller (CAN SPARK MAX) and allows for reading
encoder values.

The chassis also features a pancake piston that allows
for the transmission to change from 'torque' to 'speed'
and vice-versa.

### DragonFly
This is an important part of the drivetrain. It is an add-on
that allows us to control our robot as if it were a swerve.

It consists of a middle omni-wheel which can be moved up
or down using a piston connected to a solenoid.

The wheel has a motor (NEO) connected to it and when 
contacting the floor it allows for the robot to move with 
other wheels.

### components

- 5x CAN SPARK MAX
    - 2 per side
    - 1 for dragonfly
- 2x solenoid
    - 1 for transmission
    - 1 for dragonfly wheel

# Intake
Allows robot to use game pieces and move them around.

Consists of rollers with one motor, 2 pistons (same solenoid)
to move the rollers up and down. 

When it is down, the rollers are slightly lower than
a game piece, allowing for some compression and greater
grip on the rollers.

### components

- 1x CAN SPARK MAX
- 1x solenoid


# Transport
This subsystem takes game pieces directly from
the intake and prepares them for the shooter.

### components
- 1x Talon SRX
- 1x CAN SPARK MAX

# Shooter
It is in charge of throwing game pieces with enough
speed to score into the correct places.

The shooter also has some metal structures (moved by
two servos) which allow for the angle of the shooter
to be modified.

### components
- 2x CAN SPARK MAX
- 2x REV Smart Servo

# Turret
Changes the direction of the shooter. Around 270 degrees
of freedom.

### components
- 1x CAN SPARK 

# Climber
It literally makes the robot climb.
It uses some hangers with pistons connected to it.
It climbs like monkeys.

### components
- 2x solenoids
- 2x CAN SPARK MAX

# Vision
It is the subsystem in charge of telling the turret
and/or shooter how to position themselves to shoot
in the correct place.

It uses computer vision to look for targets, using
their x and y offsets relative to the camera and the area
of the same targets to calculate how far the robot is.

### components
- 1x limelight (running photonvision)

# TODO
Add some kind of interface class
that adds safety and checks for each subsystem.
