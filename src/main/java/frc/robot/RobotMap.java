//*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.resources.TecbotSpeedController.TypeOfMotor;

public class RobotMap {


    public static final int CLIMBER_RIGHT_LIMIT_SWITCH_PORT = 0;
    public static final int CLIMBER_LEFT_LIMIT_SWITCH_PORT = 0;

    // PCM
    private static final int PCM_1_PORT = 21;
    private static final int PCM_2_PORT = 2;
    private static final PneumaticsModuleType
            PCM_TYPE = PneumaticsModuleType.REVPH;
    // PCM end


    // DRIVETRAIN

        //chassis
    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_PORTS = {1, 2};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_PORTS = {9, 10};

        //chassis motor types
    public static final TypeOfMotor[] DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};
    public static final TypeOfMotor[] DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS};


        //chassis inverted motors;
    /*
     * If any of the motors of the chassis must be inverted,
     * indicate the port(s) in these arrays.
     */
    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_INVERTED_MOTORS = {1, 2};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_INVERTED_MOTORS = {9, 10};



        // encoders
        // specify which motors are responsible for each side of the robot odometry.
    /*
     * If encoder is connected to speed controller,
     * indicate the speed controller port here, and put encoder ports in
     * config not set
     */
    public static final int DRIVE_TRAIN_LEFT_CHASSIS_MOTOR_WITH_ENCODER = 1;
    public static final int DRIVE_TRAIN_RIGHT_CHASSIS_MOTOR_WITH_ENCODER = 9;
        //  ... dragonfly
    public static final int DRIVE_TRAIN_MIDDLE_CHASSIS_MOTOR_WITH_ENCODER = RobotConfigurator.CONFIG_NOT_SET;

        // chassis encoder-specific
    /*
     * If encoder is connected to RoboRIO,
     * indicate the port here, and put encoder motor ports in
     * config not set
     */
    public static final int[] DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};
    public static final int[] DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};

        //  ... dragonfly
    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_ENCODER_PORTS = {RobotConfigurator.CONFIG_NOT_SET, RobotConfigurator.CONFIG_NOT_SET};

    // encoders specify if inverted
    public static final boolean DRIVE_TRAIN_LEFT_CHASSIS_ENCODER_IS_INVERTED = true;
    public static final boolean DRIVE_TRAIN_RIGHT_CHASSIS_ENCODER_IS_INVERTED = false;
    public static final boolean DRIVE_TRAIN_MIDDLE_CHASSIS_ENCODER_IS_INVERTED = true;

        // transmission
    // for toggling between 'torque' and 'speed' modes.
    //  ... solenoid
    public static final int[] DRIVE_TRAIN_TRANSMISSION_SOLENOID_PORTS = {PCM_1_PORT, 4, 5};
    public static final DoubleSolenoid.Value DRIVE_TRAIN_TORQUE_TRANSMISSION = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value DRIVE_TRAIN_SPEED_TRANSMISSION = DoubleSolenoid.Value.kReverse;

        // true if transmission available
    public static final boolean DRIVE_TRAIN_TRANSMISSION_AVAILABLE = true;


        // dragonfly

    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_PORT = {11};
    //before mechanical change the wheel was inverted.
    public static final int[] DRIVE_TRAIN_MIDDLE_WHEEL_INVERTED_MOTORS = {};
    public static final TypeOfMotor[] DRIVE_TRAIN_MIDDLE_WHEEL_MOTOR_TYPES = {TypeOfMotor.CAN_SPARK_BRUSHLESS};

    public static final int[] DRIVE_TRAIN_WHEEL_SOLENOID_PORTS = {PCM_1_PORT, 6, 7};
    public static final DoubleSolenoid.Value DRIVE_TRAIN_LOWERED_WHEEL = DoubleSolenoid.Value.kReverse;
    public static final DoubleSolenoid.Value DRIVE_TRAIN_RAISED_WHEEL = DoubleSolenoid.Value.kForward;
    public static final boolean DRIVE_TRAIN_DRAGON_FLY_IS_AVAILABLE = true;

    // DRIVETRAIN --- end


    // INTAKE

        // intake motor
    public static final int INTAKE_MOTOR_PORT = 32;
    public static final TypeOfMotor INTAKE_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS;
    public static final boolean INTAKE_MOTOR_IS_INVERTED = false;
        // intake motor should be positive when receiving balls,
        //      negative when going out of the robot.

        // solenoid
    public static final int[] INTAKE_SOLENOID_PORTS  = {9, 11};
    public static final DoubleSolenoid.Value INTAKE_POSITION_ACTIVE = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value INTAKE_POSITION_RETRACTED = DoubleSolenoid.Value.kReverse;

    // INTAKE --- end

    // TRANSPORT
    public static final int[] TRANSPORT_MOTOR_PORTS = {7, 8};
    public static final TypeOfMotor[] TRANSPORT_MOTOR_TYPES = { TypeOfMotor.TALON_SRX, TypeOfMotor.CAN_SPARK_BRUSHLESS};
    public static final boolean[] TRANSPORT_MOTORS_ARE_INVERTED = { false, false };

    // TRANSPORT --- end

    // SHOOTER
    public static final int[] SHOOTER_MOTOR_PORTS = { 30, 31} ;
    public static final TypeOfMotor[] SHOOTER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS} ;
    public static final boolean[] SHOOTER_MOTORS_ARE_INVERTED = { false, false } ;

    public static final int[] SHOOTER_ANGLE_SERVO_PORTS = { 0, 1 };
    public static final boolean[] SHOOTER_ANGLE_SERVOS_ARE_INVERTED = { false, true } ;

    // SHOOTER --- end

    // TURRET
        // should go clockwise when in positive.
    public static final int[] TURRET_MOTOR_PORT = { 29 } ;
    public static final TypeOfMotor TURRET_MOTOR_TYPE = TypeOfMotor.CAN_SPARK_BRUSHLESS ;
    public static final boolean TURRET_MOTOR_IS_INVERTED = false;

    // TURRET --- end

    // CLIMBER
    public static final int[] CLIMBER_MOTOR_PORTS = {40, 41 };
    public static final TypeOfMotor[] CLIMBER_MOTOR_TYPES = { TypeOfMotor.CAN_SPARK_BRUSHLESS, TypeOfMotor.CAN_SPARK_BRUSHLESS } ;
    public static final boolean[] CLIMBER_MOTORS_ARE_INVERTED = { false, false } ;

    // in theory 2 pistons, but just one solenoid
    public static final int[] CLIMBER_SOLENOID_PORTS = { 9, 10} ;
    public static final DoubleSolenoid.Value CLIMBER_SOLENOID_EXTENDED_POSITION = DoubleSolenoid.Value.kForward;
    public static final DoubleSolenoid.Value CLIMBER_SOLENOID_RETRACTED_POSITION = DoubleSolenoid.Value.kReverse;

    // CLIMBER --- end


    // VISION

        // dont know what to add here... yet

    // VISION --- end

    public static final boolean SHOOTER_ENCODER_IN_RIGHT_MOTOR = false;
    // SHOOTER


    // INTAKE
}
