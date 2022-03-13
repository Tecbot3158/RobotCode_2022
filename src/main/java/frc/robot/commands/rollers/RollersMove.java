// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rollers;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Rollers;

public class RollersMove extends CommandBase {

    Rollers rollers;

    /**
     * Creates a new IntakeAbsorb.
     */
    public RollersMove() {

        rollers = Robot.getRobotContainer().getRollers();
        addRequirements(rollers);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = OI.getInstance().getRollersInput();

        rollers.setRaw(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        rollers.setRaw(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
