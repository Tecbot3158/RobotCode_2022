// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class MoveTurretToTarget extends CommandBase {
  /** Creates a new MoveTurretToTarget. */
  public MoveTurretToTarget() {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(Robot.getRobotContainer().getTurret());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double yaw = Robot.getRobotContainer().getTecbotcamera().getYaw();
    Robot.getRobotContainer().getTurret().settoTarget(yaw);

    if (Robot.getRobotContainer().getTurret().getTurretPID().atSetpoint()) {

      end(true);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
