package frc.robot.commands.compound;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.transport.Feeder;

public class ToggleFeederAndShooter extends CommandBase {

    Feeder feeder;
    Shooter shooter;

    public ToggleFeederAndShooter() {

        feeder = Robot.getRobotContainer().getFeeder();
        shooter = Robot.getRobotContainer().getShooter();

        addRequirements(feeder, shooter);

    }

    @Override
    public void initialize() {

//        Robot.debug(" 000 status: " + Feeder.state.toString());
//        if (Feeder.state == FeederAndShooterState.BOTH_OFF ){
//            new SetFeederAndShooter().schedule(true);
//
//            System.out.println(" status == both_off: " + Feeder.state.toString());
//            System.out.println(" ONNN");
//        }
//        else{
//            new FeederStop().schedule(true);
//            new ShooterOff().schedule(true);
//
//            System.out.println(" OFFF");
//            System.out.println(" status: " + Feeder.state.toString());
//        }
        Robot.debug("init togglefeederandshooter");
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
