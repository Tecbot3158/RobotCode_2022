// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.misc.oitests;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;

public class OITestTriggers extends CommandBase {

    OI oi;

    /**
     * Creates a new IntakeAbsorb.
     */
    public OITestTriggers() {

        oi = OI.getInstance();
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // feeder.feederSetDefaultPIDReference(feeder.getFeederPIDController());
        System.out.println("Triggers; " + oi.getPilot().getTriggers());
        System.out.println("Left Trigger; " + oi.getPilot().getLeftTrigger());
        System.out.println("Right Trigger; " + oi.getPilot().getRightTrigger());

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // feeder.setRaw(0);

        System.out.println("OUTTT!");




    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
