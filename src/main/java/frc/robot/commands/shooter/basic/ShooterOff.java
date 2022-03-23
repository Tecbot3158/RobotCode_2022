package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterOff extends CommandBase {

    Shooter shooter;

    public ShooterOff() {
        shooter = Robot.getRobotContainer().getShooter();

        addRequirements(shooter);

    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

        shooter.setShooterMotorsRaw(0);
        Robot.debug("Exec ShooterOff");
//        Feeder.state = FeederAndShooterState.SHOOTER_OFF;

    }

    @Override
    public void end(boolean interrupted) {

        //       if (Feeder.state == FeederAndShooterState.FEEDER_OFF )
        //           Feeder.state = FeederAndShooterState.BOTH_OFF;
        //       else
        //           Feeder.state = FeederAndShooterState.SHOOTER_OFF;


    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
