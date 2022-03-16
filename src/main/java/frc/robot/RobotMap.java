//*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

public class RobotMap {

        public static final int CLIMBER_RIGHT_LIMIT_SWITCH_PORT = 0;
        public static final int CLIMBER_LEFT_LIMIT_SWITCH_PORT = 0;

        // PCM
        private static final int PCM_1_PORT = 50;

        private static final PneumaticsModuleType PCM_TYPE = PneumaticsModuleType.REVPH;
        // PCM end

        // DRIVETRAIN

        // chassis
        public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_PORTS = { 10, 9 };
        public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_PORTS = { 1, 2 };

        // chassis motor types
        public static final TypeOfMotor[] DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };
        public static final TypeOfMotor[] DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };

        // chassis inverted motors;
        /*
         * If any of the motors of the chassis must be inverted,
         * indicate the port(s) in these arrays.
         */
        public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_INVERTED_MOTORS = { 10, 9 };
        public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_INVERTED_MOTORS = {};

        // encoders
        // specify which motors are responsible for each side of the robot odometry.
        /*
         * If encoder is connected to speed controller,
         * indicate the speed controller port here, and put encoder ports in
         * config not set
         */
        public static final int DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_WITH_ENCODER = 1;
        public static final int DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_WITH_ENCODER = 9;
        // ... dragonfly
        public static final int DRIVE_TRAIN_MIDDLE_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;

        // chassis encoder-specific
        /*
         * If encoder is connected to RoboRIO,
         * indicate the port here, and put encoder motor ports in
         * config not set
         */
        public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS = { RobotConfigurator.CONFIG_NOT_SET,
                        RobotConfigurator.CONFIG_NOT_SET };
        public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS = { RobotConfigurator.CONFIG_NOT_SET,
                        RobotConfigurator.CONFIG_NOT_SET };

        // ... dragonfly
        public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS = { RobotConfigurator.CONFIG_NOT_SET,
                        RobotConfigurator.CONFIG_NOT_SET };

        // encoders specify if inverted
        public static final boolean DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_IS_INVERTED = true;
        public static final boolean DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_IS_INVERTED = false;
        public static final boolean DRIVE_TRAIN_MIDDLE_CHASSIS_ENCODER_IS_INVERTED = true;

        // transmission
        // for toggling between 'torque' and 'speed' modes.
        // ... solenoid
        public static final int[] DRIVE_TRAIN_TRANSMISSION_SOLENOID_PORTS = { PCM_1_PORT, 4, 5 };
        public static final DoubleSolenoid.Value DRIVE_TRAIN_TORQUE_TRANSMISSION = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value DRIVE_TRAIN_SPEED_TRANSMISSION = DoubleSolenoid.Value.kReverse;

        // true if transmission available
        public static final boolean DRIVE_TRAIN_TRANSMISSION_AVAILABLE = true;

        // dragonfly

        public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_PORT = { 11 };
        // before mechanical change the wheel was inverted.
        public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_INVERTED_MOTORS = { 11 };
        public static final TypeOfMotor[] DRIVE_TRAIN_MIDDLE_WHEEL_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS };

        public static final int[] DRIVE_TRAIN_WHEEL_SOLENOID_PORTS = { PCM_1_PORT, 6, 7 };
        public static final DoubleSolenoid.Value DRIVE_TRAIN_LOWERED_WHEEL = DoubleSolenoid.Value.kReverse;
        public static final DoubleSolenoid.Value DRIVE_TRAIN_RAISED_WHEEL = DoubleSolenoid.Value.kForward;
        public static final boolean DRIVE_TRAIN_DRAGON_FLY_IS_AVAILABLE = true;

        // DRIVETRAIN --- end

        // INTAKE

        // intake motor
        public static final int INTAKE_MOTOR_PORT = 32;
        public static final TypeOfMotor INTAKE_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
        public static final boolean INTAKE_MOTOR_IS_INVERTED = false;
        public static final double INTAKE_ABSORB_SPEED = 0.8;
        public static final double INTAKE_EJECT_SPEED = -0.8;
        public static final double INTAKE_OFF = 0;
        // intake motor should be positive when receiving balls,
        // negative when going out of the robot.

        // solenoid
        public static final int[] INTAKE_SOLENOID_PORTS = { PCM_1_PORT, 9, 11 };
        public static final DoubleSolenoid.Value INTAKE_POSITION_ACTIVE = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value INTAKE_POSITION_RETRACTED = DoubleSolenoid.Value.kReverse;

        // INTAKE --- end

        // TRANSPORT
        // ROLLERS
        public static final int TRANSPORT_ROLLER_PORT = 7;
        public static final TypeOfMotor TRANSPORT_ROLLER_MOTOR_TYPE = TypeOfMotor.TALON_SRX;
        public static final boolean TRANSPORT_ROLLER_IS_INVERTED = false;

        // FEEDER
        public static final int TRANSPORT_FEEDER_PORT = 8;
        public static final TypeOfMotor TRANSPORT_FEEDER_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
        public static final boolean TRANSPORT_FEEDER_IS_INVERTED = false;

        public static final int FEEDER_ENCODER_MOTOR_PORT = TRANSPORT_FEEDER_PORT;

        public static final double FEEDER_PID_kP = 6e-5;
        public static final double FEEDER_PID_kI = 0;
        public static final double FEEDER_PID_kD = 0;
        public static final double FEEDER_PID_kIz = 0;
        public static final double FEEDER_PID_kFF = 0.000015;
        public static final double FEEDER_PID_kMaxOutput = 1;
        public static final double FEEDER_PID_kMinOutput = -1;
        public static final double FEEDER_PID_kMaxRPM = 6969;

        public static final double FEEDER_PID_Target = 6696;

        // TRANSPORT --- end

        // SHOOTER
        public static final int[] SHOOTER_MOTOR_PORTS = { 30, 31 };
        public static final TypeOfMotor[] SHOOTER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };
        public static final boolean[] SHOOTER_MOTORS_ARE_INVERTED = { false, false };
        public static final int[] SHOOTER_INVERTED_MOTORS = {};

        public static final int[] SHOOTER_ANGLE_SERVO_PORTS = { 0, 1 };
        public static final boolean[] SHOOTER_ANGLE_SERVOS_ARE_INVERTED = { false, true };

        public static final int SHOOTER_ENCODER_MOTOR_PORT = SHOOTER_MOTOR_PORTS[0];

        public static final int SHOOTER_MOTOR_MASTER_PORT = SHOOTER_MOTOR_PORTS[0];
        public static final int SHOOTER_MOTOR_SLAVE_PORT = SHOOTER_MOTOR_PORTS[1];

        public static final double SHOOTER_PID_kP = 6e-5;
        public static final double SHOOTER_PID_kI = 0;
        public static final double SHOOTER_PID_kD = 0;
        public static final double SHOOTER_PID_kIz = 0;
        public static final double SHOOTER_PID_kFF = 0.000015;
        public static final double SHOOTER_PID_kMaxOutput = 1;
        public static final double SHOOTER_PID_kMinOutput = -1;
        public static final double SHOOTER_PID_kMaxRPM = 6969;

        public static final double SHOOTER_PID_Target = 6696;

        // SHOOTER --- end

        // TURRET
        // should go clockwise when in positive.
        public static final int TURRET_MOTOR_PORT = 29;
        public static final TypeOfMotor TURRET_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
        public static final boolean TURRET_MOTOR_IS_INVERTED = false;
        public static final int[] TURRET_ENCODER_CHANNELS = { RobotConfigurator.CONFIG_NOT_SET,
                        RobotConfigurator.CONFIG_NOT_SET };

        public static final double TURRET_P = 0.8;
        public static final double TURRET_I = 0;
        public static final double TURRET_D = 0;

        public static final double TURRET_PID_kP = 0.1;
        public static final double TURRET_PID_kI = 1e-4;
        public static final double TURRET_PID_kD = 1;
        public static final double TURRET_PID_kIz = 0;
        public static final double TURRET_PID_kFF = 0;
        public static final double TURRET_PID_kMaxOutput = 1;
        public static final double TURRET_PID_kMinOutput = -1;
        // public static final double TURRET_PID_kMaxRPM = 6969;

        public static final double TURRET_PID_Target = 6696;

        public static final double TURRET_MAX_DISTANCE = ((double) 776 / (double) 27);

        public static final double TURRET_ROTATION_TO_MOTOR_ROTATION = (double) 776 / (double) 27;
        // TURRET --- end

        // CLIMBER
        public static final int[] CLIMBER_MOTOR_PORTS = { 50, 51 };
        public static final TypeOfMotor[] CLIMBER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS,
                        TypeOfMotor.CAN_SPARK_BRUSHLESS };
        public static final int[] CLIMBER_INVERTED_MOTORS = {};

        // in theory 2 pistons, but just one solenoid
        public static final int[] CLIMBER_SOLENOID_PORTS = { PCM_1_PORT, 9, 10 };
        public static final DoubleSolenoid.Value CLIMBER_SOLENOID_EXTENDED_POSITION = DoubleSolenoid.Value.kForward;
        public static final DoubleSolenoid.Value CLIMBER_SOLENOID_RETRACTED_POSITION = DoubleSolenoid.Value.kReverse;

        public static final double CLIMBER_PID_kP = 6e-5;
        public static final double CLIMBER_PID_kI = 0;
        public static final double CLIMBER_PID_kD = 0;
        public static final double CLIMBER_PID_kIz = 0;
        public static final double CLIMBER_PID_kFF = 0.000015;
        public static final double CLIMBER_PID_kMaxOutput = 1;
        public static final double CLIMBER_PID_kMinOutput = -1;
        public static final double CLIMBER_PID_kMaxRPM = 6969;

        public static final double CLIMBER_PID_Target = 6500;

        // CLIMBER --- end

        // VISION

        public static String TECBOT_CAMERA_NAME = "limelight";

        // VISION --- end

        // INTAKE
}
