package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterGoToTarget extends CommandBase {

    StepControl stepControl;

    Shooter shooter;

    double target, currentPos;

    double increment = 0.0000045;

    double minout = 0.0001;

    public ShooterGoToTarget() {

        shooter = Robot.getRobotContainer().getShooter();

        shooter.getShooterEncoder().setPosition(0);
        currentPos = shooter.getShooterEncoder().getVelocity();
        target = 3600;

        stepControl = new StepControl(minout, target, currentPos, increment);

        Subsystem requirements[] = { shooter };

        addRequirements(requirements);
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

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
