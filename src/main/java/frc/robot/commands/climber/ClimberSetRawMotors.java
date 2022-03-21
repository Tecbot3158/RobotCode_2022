// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.climber.Climber;

public class ClimberSetRawMotors extends CommandBase {

    Climber climber;

    /**
     * Creates a new IntakeAbsorb.
     */
    public ClimberSetRawMotors() {

        climber = Robot.getRobotContainer().getClimber();
        addRequirements(climber);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putBoolean("climber done", false);

        Robot.debug("init climber Set To speed");

        climber.getRopeControllerEncoder().setPosition(0);


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double input = OI.getInstance().getClimberDefaultManualInput();

        double speed = Math.clamp(input * 0.6, -0.6, 0.6);

        double position = climber.getRopeControllerEncoder().getPosition();

        SmartDashboard.putNumber("CLIMB-speed", speed);
        SmartDashboard.putNumber("CLIMB-encoder", position);

        climber.setRopeControllerRaw( speed );

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
