// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.chassis.drivingModes;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;

public class ToggleInputState extends InstantCommand {
    /** Creates a new ToggleDragonFlyWheel. */
    public ToggleInputState() {
        addRequirements(Robot.getRobotContainer().getDriveTrain());
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        Robot.getRobotContainer().getDriveTrain().toggleInputMultiplier();

    }
}
