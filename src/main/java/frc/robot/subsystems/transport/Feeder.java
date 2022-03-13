// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

public class Feeder extends SubsystemBase {

    TecbotSpeedController feeder;


    private SparkMaxPIDController feederPIDController;
    private RelativeEncoder feederEncoder;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double feederPIDTarget;


    /**
     * Creates a new Feeder.
     */
    public Feeder() {
        feeder = new TecbotSpeedController(RobotMap.TRANSPORT_FEEDER_PORT, RobotMap.TRANSPORT_FEEDER_MOTOR_TYPE);
        feeder.setInverted(RobotMap.TRANSPORT_FEEDER_IS_INVERTED);


        initPID();


    }

    public void initPID(){
        CANSparkMax motor = feeder.getCANSparkMax();

        feederEncoder = motor.getEncoder();
        feederPIDController = motor.getPIDController();

        kP = RobotMap.FEEDER_PID_kP;
        kI = RobotMap.FEEDER_PID_kI;
        kD = RobotMap.FEEDER_PID_kD;
        kIz = RobotMap.FEEDER_PID_kIz;
        kFF = RobotMap.FEEDER_PID_kFF;
        kMaxOutput = RobotMap.FEEDER_PID_kMaxOutput;
        kMinOutput = RobotMap.FEEDER_PID_kMinOutput;
        maxRPM = RobotMap.FEEDER_PID_kMaxRPM;

        feederPIDTarget = RobotMap.FEEDER_PID_Target;

        // only when actually wanting the motors to move
        // setPIDController(feederPIDController);
    }

    public void setPIDController(SparkMaxPIDController pidController) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public SparkMaxPIDController getFeederPIDController(){
        return feederPIDController;
    }

    public TecbotSpeedController getFeederMotor() {
        return feeder;
    }

    public void setRaw(double speed){
        feeder.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
