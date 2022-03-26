package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.StepControl;
import frc.robot.resources.TecbotConstants;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterGoToLower extends CommandBase {

    StepControl stepControl;

    Shooter shooter;

    double target, currentPos;

    double increment = 0.0000045;

    double minout = 0.0001;

    public ShooterGoToLower() {

        shooter = Robot.getRobotContainer().getShooter();

        shooter.getShooterEncoder().setPosition(0);
        currentPos = shooter.getShooterEncoder().getVelocity();
        //target = 1700;
        //target = 1260;
        target = 1310;
        target = 1500;
        target = 1600;
        target = 1900;

        stepControl = new StepControl(minout, target, currentPos, increment);

        Subsystem requirements[] = {shooter};

        addRequirements(requirements);

        stepControl.setRange(RobotMap.SHOOTER_DEFAULT_RANGE);
    }

    @Override
    public void initialize() {

        SmartDashboard.putNumber("setminout", minout);
        SmartDashboard.putNumber("settarget", target);
        SmartDashboard.putNumber("setincr", increment);

        Robot.debug("init shootergototarget");

    }

    @Override
    public void execute() {

        double output = stepControl.getOutputVelocity(shooter.getShooterEncoder().getVelocity());
        double velocity = shooter.getShooterEncoder().getVelocity();
        // in rpms

        SmartDashboard.putNumber("shooter encoder. ", shooter.getShooterEncoder().getPosition());
        SmartDashboard.putNumber("shooter output ", output);
        SmartDashboard.putNumber("shooter RPMs", velocity);
        SmartDashboard.putNumber("shooter target", target);

        shooter.setShooterMotorsRaw(output);

        // minout = SmartDashboard.getNumber("setminout", minout);
        // target = SmartDashboard.getNumber("settarget", target);
        // increment = SmartDashboard.getNumber("setincr", increment);
        //
        // stepControl.setTarget(target);
        // stepControl.setIncrementMultiplier(increment);
        // stepControl.setMinAbsoluteOutput(minout);

        Robot.debug("exec shootergototarget");

        if (stepControl.isInRange()) {
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, TecbotConstants.COPILOT_DEFAULT_VIBRATION);
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, TecbotConstants.COPILOT_DEFAULT_VIBRATION);
        } else {
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);

        }


    }

    @Override
    public void end(boolean interrupted) {
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        OI.getInstance().getCopilot().setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
