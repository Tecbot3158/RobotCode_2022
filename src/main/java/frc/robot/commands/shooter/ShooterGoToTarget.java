package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.getNumber;

public class ShooterGoToTarget extends CommandBase {

    StepControl stepControl;

    Shooter shooter;

    double target, currentPos;

    double increment = 0.000003;

    double minout = 0.0001;

    public ShooterGoToTarget(){

        shooter = Robot.getRobotContainer().getShooter();

       shooter.getShooterEncoder().setPosition(0);
        currentPos = shooter.getShooterEncoder().getVelocity();
       target = 100;






        stepControl = new StepControl(minout, target, currentPos, increment);
    }

    @Override
    public void initialize() {

        SmartDashboard.putNumber("setminout", minout);
        SmartDashboard.putNumber("settarget", target);
        SmartDashboard.putNumber("setincr", increment);


    }

    @Override
    public void execute(){

       double output = stepControl.getOutput( shooter.getShooterEncoder().getVelocity() );
       double velocity = shooter.getShooterEncoder().getVelocity();
       // in rpms

        SmartDashboard.putNumber("shooter encoder. ", shooter.getShooterEncoder().getPosition());
        SmartDashboard.putNumber("output shooter ", output);
        SmartDashboard.putNumber("RPMs", velocity);

        shooter.setShooterMotorsRaw(output);

        SmartDashboard.getNumber("setminout", 0);

        minout = SmartDashboard.getNumber("setminout", minout);
        target = SmartDashboard.getNumber("settarget", target);
        increment =  SmartDashboard.getNumber("setincr", increment);

        stepControl.setTarget(target);
        stepControl.setIncrementMultiplier(increment);
        stepControl.setMinAbsoluteOutput(minout);



    }

    @Override
    public void end(boolean interrupted) {
        //shooter.setShooterMotorsRaw(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
