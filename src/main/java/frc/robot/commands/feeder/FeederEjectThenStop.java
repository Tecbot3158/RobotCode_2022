// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.transport.Feeder;

public class FeederEjectThenStop extends CommandBase {

    Feeder feeder;

    /**
     * Creates a new IntakeAbsorb.
     */
    public FeederEjectThenStop() {

        feeder = Robot.getRobotContainer().getFeeder();
        addRequirements(feeder);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        feeder.setRaw(RobotMap.FEEDER_DEFAULT_EJECT_SPEED);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        feeder.setRaw(0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
