// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSpeedController;

public class Turret extends SubsystemBase {
    /**
     * Creates a new Turret.
     */

    TecbotSpeedController turretMotor;
    TecbotEncoder turretEncoder;

    public Turret() {

        turretMotor = new TecbotSpeedController(RobotMap.TURRET_MOTOR_PORT, RobotMap.TURRET_MOTOR_TYPE);
        turretMotor.setInverted(RobotMap.TURRET_MOTOR_IS_INVERTED);

        turretMotor.getCANSparkMax().restoreFactoryDefaults();
        turretMotor.getCANSparkMax().setIdleMode(CANSparkMax.IdleMode.kBrake);

        turretEncoder = RobotConfigurator.buildEncoder(turretMotor, RobotMap.TURRET_ENCODER_CHANNELS[0], RobotMap.TURRET_ENCODER_CHANNELS[1]);

        // turretMotor.getCANSparkMax().getEncoder().setInverted(RobotMap.TURRET_ENCODER_IS_INVERTED);

        turretMotor.getCANSparkMax().getEncoder().setPosition(0);

    }

    /**
     * Moves Turret Manually
     *
     * @param speed speed for turret
     */
    public void setTurretRaw(double speed) {

        turretMotor.set(Math.clamp(speed, -0.3, 0.3));


    }

    public RelativeEncoder getTurretEncoder (){
        return turretMotor.getCANSparkMax().getEncoder();
    }


    /**
     * get number of rotations converted from an angle in degrees.
     * Assumes that the turret at its center is at 0 deg.
     * To the left it's negative degrees and positive to the right.
     *
     * @return number of rotations based on angle.
     */
    public double getRotationsFromAngle(double angleDegrees) {

        return angleDegrees / (double) 360 * RobotMap.TURRET_ROTATION_TO_MOTOR_ROTATION;

    }


    public void setTurretMoveRight() {
        turretMotor.set(0.1);
    }

    public void setTurretMoveLeft() {
        turretMotor.set(-0.1);
    }
}
