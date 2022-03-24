// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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

        turretEncoder = RobotConfigurator.buildEncoder(turretMotor, RobotMap.TURRET_ENCODER_CHANNELS[0],
                RobotMap.TURRET_ENCODER_CHANNELS[1]);

        // turretMotor.getCANSparkMax().getEncoder().setInverted(RobotMap.TURRET_ENCODER_IS_INVERTED);

        turretMotor.getCANSparkMax().getEncoder().setPosition(0);

    }

    /**
     * Moves Turret Manually
     *
     * @param speed speed for turret
     */
    public void setTurretRaw(double speed) {
        double clampedSpeed = Math.clamp(speed, RobotMap.TURRET_MINIMUM_SPEED, RobotMap.TURRET_MAXIMUM_SPEED);
        double realSpeed = clampedSpeed;

        if (!inRangeNegativeEncoderValue())
            realSpeed = Math.clamp(clampedSpeed, -1, 0);
        if (!inRangePositiveEncoderValue())
            realSpeed = Math.clamp(clampedSpeed, 0, 1);

        turretMotor.set(realSpeed);

//        <= minimo clampear para solo negativos entre -1 y 0
        // si es mayor al valor mayor, potencia del setRaw a 0 y 1;

    }

    public boolean inRangeNegativeEncoderValue() {
        RelativeEncoder encoder = turretMotor.getCANSparkMax().getEncoder();

        return encoder.getPosition() >= RobotMap.TURRET_DEFAULT_MINIMUM_ENCODER_VALUE;

    }

    public boolean inRangePositiveEncoderValue() {
        RelativeEncoder encoder = turretMotor.getCANSparkMax().getEncoder();

        return encoder.getPosition() <= RobotMap.TURRET_DEFAULT_MAXIMUM_ENCODER_VALUE;

    }

    public RelativeEncoder getTurretEncoder() {
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

    public double getAngleFromRotations(double rotations){
        return rotations * RobotMap.TURRET_ROTATION_TO_MOTOR_ROTATION / 360;
    }

    public void setTurretMoveRight() {
        setTurretRaw(RobotMap.TURRET_MANUAL_POSITIVE_SPEED);
    }

    public void setTurretMoveLeft() {
        setTurretRaw(RobotMap.TURRET_MANUAL_NEGATIVE_SPEED);
    }

    public double getAngleMaxRangeCounterClockwise() {
        return RobotMap.TURRET_ANGLE_COUNTER_CLOCKWISE;
    }

    public double getAngleMinRangeClockwise() {
        return RobotMap.TURRET_ANGLE_CLOCKWISE;
    }

    public boolean withinAngleRange(double angleDegrees) {
        if (angleDegrees >= getAngleMaxRangeCounterClockwise() && angleDegrees <= getAngleMinRangeClockwise())
            return true;
        else {
            if (angleDegrees < 0) {
                double positiveAngle = 360 + angleDegrees;
                if (positiveAngle >= getAngleMaxRangeCounterClockwise() && positiveAngle <= getAngleMinRangeClockwise())
                    return true;
            }
            if (angleDegrees > 0) {
                double negativeAngle = -360 + angleDegrees;
                if (negativeAngle >= getAngleMaxRangeCounterClockwise() && negativeAngle <= getAngleMinRangeClockwise())
                    return true;
            }
        }

        return false;

    }

    public double getRealAngle(double angleDegrees) {
        if (withinAngleRange(angleDegrees)) {
            if (!(angleDegrees >= getAngleMaxRangeCounterClockwise() && angleDegrees <= getAngleMinRangeClockwise())) {
                if (angleDegrees > 0)
                    return -360 + angleDegrees;
                else
                    return 360 + angleDegrees;
            } else
                return angleDegrees;
        }

        return 0;

    }

    public double getPosition() {
        return getTurretEncoder().getPosition();
    }
}
