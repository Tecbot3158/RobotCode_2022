// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotEncoder;
import frc.robot.resources.TecbotSpeedController;

public class Turret extends SubsystemBase {
    /**
     * Creates a new Turret.
     */

    TecbotSpeedController turretmotor;
    TecbotEncoder turretencoder;

    PIDController turretpid;

    public Turret() {

        turretmotor = new TecbotSpeedController(RobotMap.TURRET_MOTOR_PORT[0], RobotMap.TURRET_MOTOR_TYPE);
        turretencoder = new TecbotEncoder(RobotMap.TURRET_ENCODER_CHANNELS[0], RobotMap.TURRET_ENCODER_CHANNELS[1]);
        turretpid = new PIDController(RobotMap.TURRET_P, RobotMap.TURRET_I, RobotMap.TURRET_D);

    }

    /**
     * Moves Turret Manually
     *
     * @param speed Left Speed
     */
    public void setTurretRaw(double speed) {

        turretmotor.set(speed);

    }

    /***
     * Sets Turret to an angle from 0 to 270 Robot Centered using a PID Controller
     *
     * @param angle The angle that the turret is going to move towards
     */
    public void settoAngle(double angle) {

        if (angle > 270) {

            angle = 270;

        } else if (angle < 0) {
            angle = 0;
        }

        double setpoint = ((angle - 135) / 180) * RobotMap.TURRET_MAX_DISTANCE;

        turretpid.setSetpoint(setpoint);
        turretmotor.set(turretpid.calculate(turretencoder.getRaw(), setpoint));

    }

    /***
     * Sets Turret to a Target.
     * This method should be set to a photonvision target
     *
     * @param yaw PhotonVision Camera raw
     */
    public void settoTarget(double yaw) {

        double setpoint = 0.0;
        turretpid.setSetpoint(setpoint);

        turretmotor.set(turretpid.calculate(yaw, setpoint));

    }

    public PIDController getTurretPID() {

        return turretpid;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
