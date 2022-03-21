// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.transport.Feeder;

public class ClimberRopeStartFirstShot extends CommandBase {

    Climber climber;

    StepControl stepControl;

    double kMinimumAbsOutput, target, currentPosition, kIncrementMultiplier;

    /**
     * Creates a new IntakeAbsorb.
     */
    public ClimberRopeStartFirstShot() {

        climber = Robot.getRobotContainer().getClimber();
        addRequirements(climber);

        kMinimumAbsOutput = RobotMap.CLIMBER_DEFAULT_kMINIMUM_ABSOLUTE_OUTPUT;
        target = RobotMap.CLIMBER_DEFAULT_TARGET_SPEED;
        currentPosition = climber.getRopeControllerEncoder().getVelocity();
        kIncrementMultiplier = RobotMap.CLIMBER_DEFAULT_kINCREMENT_MULTIPLIER;
        stepControl = new StepControl( kMinimumAbsOutput, target, currentPosition, kIncrementMultiplier);
        // Use addRequirements() here to declare subsystem dependencies.

        stepControl.setRange(RobotMap.CLIMBER_DEFAULT_RANGE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putBoolean("climber done", false);

        Robot.debug("init climber Set To speed");

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }


    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
