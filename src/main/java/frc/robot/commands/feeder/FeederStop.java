// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.transport.Feeder;

public class FeederStop extends CommandBase {

    Feeder feeder;

    /**
     * Creates a new IntakeAbsorb.
     */
    public FeederStop() {

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
        Robot.debug(" exec feederstop");
        feeder.setRaw(0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

//        if (Feeder.state == FeederAndShooterState.SHOOTER_OFF )
//            Feeder.state = FeederAndShooterState.BOTH_OFF;
//        else
//            Feeder.state = FeederAndShooterState.FEEDER_OFF;

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
