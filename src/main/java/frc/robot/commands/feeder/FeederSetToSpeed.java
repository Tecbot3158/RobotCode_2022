// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.transport.Feeder;

public class FeederSetToSpeed extends CommandBase {

    Feeder feeder;

    StepControl stepControl;

    double kMinimumAbsOutput, target, currentPosition, kIncrementMultiplier;

    /**
     * Creates a new IntakeAbsorb.
     */
    public FeederSetToSpeed() {

        feeder = Robot.getRobotContainer().getFeeder();
        addRequirements(feeder);

        kMinimumAbsOutput = RobotMap.FEEDER_DEFAULT_kMINIMUM_ABSOLUTE_OUTPUT;
        target = RobotMap.FEEDER_DEFAULT_TARGET_SPEED;
        currentPosition = feeder.getFeederEncoder().getVelocity();
        kIncrementMultiplier = RobotMap.FEEDER_DEFAULT_kINCREMENT_MULTIPLIER;
        stepControl = new StepControl( kMinimumAbsOutput, target, currentPosition, kIncrementMultiplier);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putBoolean("feeder done", false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        currentPosition = feeder.getFeederEncoder().getVelocity();

        double output = stepControl.getOutput(currentPosition);

        feeder.setRaw(output);

        // feeder.feederSetDefaultPIDReference(feeder.getFeederPIDController());
        // System.out.println("FEeder set to speed!!!");
        // feeder.setRaw(0.42);

        SmartDashboard.putBoolean("feeder in RANGE.", stepControl.isInRange() );

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // feeder.setRaw(0);

        // System.out.println("OUTTT!");

        SmartDashboard.putBoolean("feeder done", true);


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
