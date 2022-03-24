// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.feeder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.resources.TecbotConstants;
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
        stepControl = new StepControl(kMinimumAbsOutput, target, currentPosition, kIncrementMultiplier);
        // Use addRequirements() here to declare subsystem dependencies.

        stepControl.setRange(RobotMap.FEEDER_DEFAULT_RANGE);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        SmartDashboard.putBoolean("feeder done", false);

        // if ( Feeder.state == FeederAndShooterState.SHOOTER_ON){
        // Feeder.state = FeederAndShooterState.BOTH_ON;
        // }
        // else {
        // Feeder.state = FeederAndShooterState.FEEDER_ON;
        // }

        Robot.debug("init feeder Set To speed");

        feeder.setRaw(1);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // if ( ! Robot.getRobotContainer().getShooter().getIsReady() )
        // stepControl.setTarget( -1000 );
        // else
        // stepControl.setTarget( RobotMap.FEEDER_DEFAULT_TARGET_SPEED );
        //
        //
        // currentPosition = feeder.getFeederEncoder().getVelocity();
        //
        // double output = stepControl.getOutputVelocity(currentPosition);
        //
        // feeder.setRaw(output);
        //
        // SmartDashboard.putBoolean("feeder in RANGE.", stepControl.isInRange() );
        // SmartDashboard.putNumber("feeder velocity: ", currentPosition);
        // SmartDashboard.putNumber("feeder output: ", output);
        // SmartDashboard.putNumber("feeder target", target);
        // SmartDashboard.putNumber("feeder kMult", kIncrementMultiplier);
        // SmartDashboard.putNumber("feeder kMinAbsPost", kMinimumAbsOutput);
        //
        // Robot.debug("exec feedersettospeed");
        //
        // double intarget = SmartDashboard.getNumber("feeder target", target) ;
        //
        // if (intarget != target ) {
        // target = intarget;
        // stepControl.setTarget(target);
        // }
        //
        // double inkmult = SmartDashboard.getNumber("feeder kMult",
        // kIncrementMultiplier) ;
        //
        // if (inkmult != kIncrementMultiplier ) {
        // kIncrementMultiplier = inkmult;
        // stepControl.setMinAbsoluteOutput(kIncrementMultiplier);
        // }
        //
        // double inkabspost = SmartDashboard.getNumber("feeder kMinAbsPost",
        // kMinimumAbsOutput) ;
        //
        // if (inkabspost != kMinimumAbsOutput ) {
        // kMinimumAbsOutput = inkmult;
        // stepControl.setMinAbsoluteOutput(kMinimumAbsOutput);
        // }
        //
        //
        // if ( stepControl.isInRange() &&
        // Robot.getRobotContainer().getShooter().getIsReady() ) {
        // feeder.setReady( true );
        // OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble,
        // TecbotConstants.COPILOT_DEFAULT_VIBRATION);
        // OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble,
        // TecbotConstants.COPILOT_DEFAULT_VIBRATION);
        // } else {
        // feeder.setReady( false ); ;
        //
        // OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble,
        // 0);
        // OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble,
        // 0);
        //
        //
        // }

        // SmartDashboard.putNumber("feeder REAL RPMS",
        // feeder.getFeederEncoder().getVelocity());

    }

    @Override
    public void end(boolean interrupted) {
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
