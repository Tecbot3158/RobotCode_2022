package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.resources.StepControl;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterSetServoSpeeds extends CommandBase {

    Shooter shooter;

    double speedLeft, speedRight;

    public ShooterSetServoSpeeds(){

        shooter = Robot.getRobotContainer().getShooter();

        addRequirements(shooter);
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute(){

        speedLeft = SmartDashboard.getNumber("SERVO LS", 0);
        speedRight = SmartDashboard.getNumber("SERVOR RS", 0);


        shooter.setServoLeft( speedLeft );
        shooter.setServoRight( speedRight );

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
