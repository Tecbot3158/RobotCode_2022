// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret.basic;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class DriveTurretRaw extends CommandBase {


    double speed = 0;

    /**
     * sets the turret to a constant speed.
     * then exits immediately.
     */
    public DriveTurretRaw(double speed) {
        this.speed = speed;
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

        Robot.getRobotContainer().getTurret().setTurretRaw(speed);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
