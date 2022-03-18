// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.transport.Feeder;

public class FeederRaw extends CommandBase {

    Feeder feeder;

    /**
     * Creates a new IntakeAbsorb.
     */
    public FeederRaw() {

        feeder = Robot.getRobotContainer().getFeeder();
        addRequirements(feeder);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // feeder.setRaw();

        // feeder.feederSetDefaultPIDReference(feeder.getFeederPIDController());
        // System.out.println("FEeder set to speed!!!");
        // feeder.setRaw(0.42);

        // SmartDashboard.putBoolean("feeder in RANGE.", stepControl.isInRange() );
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // feeder.setRaw(0);

        // System.out.println("OUTTT!");


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
