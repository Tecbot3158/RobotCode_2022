// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.intake.Intake;

public class IntakeToggleMotors extends CommandBase {

  Intake intake;

  /** Creates a new IntakeToggle. */
  public IntakeToggleMotors() {
    intake = Robot.getRobotContainer().getIntake();
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intake.getIntakeMotorState() == Intake.IntakeMotorState.ABSORB )

      intake.setIntakeMotorsState(Intake.IntakeMotorState.OFF);
    else
      intake.setIntakeMotorsState(Intake.IntakeMotorState.ABSORB);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
