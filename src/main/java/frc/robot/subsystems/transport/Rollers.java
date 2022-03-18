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
        roller.setInverted(RobotMap.TRANSPORT_ROLLER_IS_INVERTED);
    }

    public TecbotSpeedController getRollerMotor() {
        return roller;
    }

    public void setRaw(double speed) {
        roller.set(speed);
    }

    public void setAbsorb (){
        roller.set(RobotMap.TRANSPORT_ROLLERS_ABSORB_SPEED);
    }

    public void setThrow (){
        roller.set(RobotMap.TRANSPORT_ROLLERS_THROW_SPEED);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
