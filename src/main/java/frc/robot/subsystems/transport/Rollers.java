// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

public class Rollers extends SubsystemBase {

    TecbotSpeedController roller;

    /**
     * Creates a new Rollers.
     */
    public Rollers() {
        roller = new TecbotSpeedController(RobotMap.TRANSPORT_ROLLER_PORT, RobotMap.TRANSPORT_ROLLER_MOTOR_TYPE);
    }

    public TecbotSpeedController getRollerMotor() {
        return roller;
    }

    public void rollerSpin() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
