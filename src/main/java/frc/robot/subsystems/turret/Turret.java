// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSpeedController;

public class Turret extends SubsystemBase {
    /**
     * Creates a new Turret.
     */

    TecbotSpeedController turretMotor;
    TecbotEncoder turretEncoder;

    SparkMaxPIDController turretPIDController;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private double turretPIDTarget;


    public Turret() {

        turretMotor = new TecbotSpeedController(RobotMap.TURRET_MOTOR_PORT, RobotMap.TURRET_MOTOR_TYPE);
        turretMotor.setInverted(RobotMap.TURRET_MOTOR_IS_INVERTED);

        turretMotor.getCANSparkMax().restoreFactoryDefaults();
        turretMotor.getCANSparkMax().setIdleMode(CANSparkMax.IdleMode.kBrake);

        turretEncoder = RobotConfigurator.buildEncoder(turretMotor, RobotMap.TURRET_ENCODER_CHANNELS[0], RobotMap.TURRET_ENCODER_CHANNELS[1]);
        turretPIDController = turretMotor.getCANSparkMax().getPIDController();

        initPID();

        turretMotor.getCANSparkMax().getEncoder().setPosition(0);

    }

    /**
     * Moves Turret Manually
     *
     * @param speed speed for turret
     */
    public void setTurretRaw(double speed) {

        turretMotor.set(speed);


    }

    public void initPID() {
        kP = RobotMap.TURRET_PID_kP;
        kI = RobotMap.TURRET_PID_kI;
        kD = RobotMap.TURRET_PID_kD;
        kIz = RobotMap.TURRET_PID_kIz;
        kFF = RobotMap.TURRET_PID_kFF;

        kMaxOutput = RobotMap.TURRET_PID_kMaxOutput;
        kMinOutput = RobotMap.TURRET_PID_kMinOutput;

        setPIDController(turretPIDController);

    }

    public void setPIDController(SparkMaxPIDController pidController) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public REVLibError setReferencePIDController(double rotations) {
        return turretPIDController.setReference(rotations, CANSparkMax.ControlType.kPosition);
    }

    public SparkMaxPIDController getTurretPIDController() {
        return turretPIDController;
    }

    /**
     * get number of rotations converted from an angle in degrees.
     * Assumes that the turret at its center is at 0 deg.
     * To the left it's negative degrees and positive to the right.
     *
     * @return number of rotations based on angle.
     */
    private double getRotationsFromAngle(double angleDegrees) {

        return angleDegrees / (double) 360 * RobotMap.TURRET_ROTATION_TO_MOTOR_ROTATION;

    }


}
