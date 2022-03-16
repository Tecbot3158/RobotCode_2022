package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterGoRaw extends CommandBase {

    StepControl stepControl;

    Shooter shooter;

    double target, currentPos;

    double increment;

    double speed;

    public ShooterGoRaw(double num){

        speed = num;

        shooter = Robot.getRobotContainer().getShooter();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){

        SmartDashboard.putNumber("shooter encoder. ", shooter.getShooterEncoder().getPosition());
        SmartDashboard.putNumber("output shooter ", speed);

        shooter.setShooterMotorsRaw(speed);

    }

    @Override
    public void end(boolean interrupted) {

        shooter.setShooterMotorsRaw(0);

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
