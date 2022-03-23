// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.subsystems.climber.Climber;

public class ClimberToggleSolenoids extends CommandBase {

    Climber climber;

    /**
     * Creates a new IntakeAbsorb.
     */
    public ClimberToggleSolenoids() {

        climber = Robot.getRobotContainer().getClimber();
        addRequirements(climber);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (climber.getPistonValue() == RobotMap.CLIMBER_SOLENOID_EXTENDED_POSITION) {
            climber.setPistonRetract();
            SmartDashboard.putBoolean("CLIMB-PISTON", false);
        }
        else {
            climber.setPistonExtend();
            SmartDashboard.putBoolean("CLIMB-PISTON", true);
        }


    }


    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
