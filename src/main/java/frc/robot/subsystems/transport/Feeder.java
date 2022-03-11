// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.resources.TecbotSpeedController;
import frc.robot.RobotMap;

public class Feeder extends SubsystemBase {

  TecbotSpeedController feeder;

  /** Creates a new Feeder. */
  public Feeder() {
    feeder = new TecbotSpeedController(RobotMap.TRANSPORT_FEEDER_PORT, RobotMap.TRANSPORT_FEEDER_MOTOR_TYPE);
  }

  public TecbotSpeedController getFeederMotor() {
    return feeder;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
