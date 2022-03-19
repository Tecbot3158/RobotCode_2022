package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.chassis.drivingModes.ToggleMecanum;
import frc.robot.commands.compound.SetFeederAndShooter;
import frc.robot.commands.compound.ToggleFeederAndShooter;
import frc.robot.commands.feeder.FeederRaw;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.intake.IntakeDefault;
import frc.robot.commands.intake.basic.*;
import frc.robot.commands.oitests.OITestTriggers;
import frc.robot.commands.rollers.RollersMove;
import frc.robot.commands.rollers.RollersRunThenStop;
import frc.robot.commands.rollers.SetRollersRaw;
import frc.robot.commands.shooter.ShooterGoRaw;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.turret.DriveTurretManually;
import frc.robot.commands.turret.DriveTurretRaw;
import frc.robot.commands.turret.MoveTurretToCenter;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;
import frc.robot.subsystems.chassis.DriveTrain;
import frc.robot.subsystems.shooter.Shooter;

public class OI {

    public static OI instance;

    private TecbotController pilot, copilot;

    private OI() {

    }

    /**
     * must always be called after ALL subsystems are
     * created.
     */
    public void configureButtonBindings() {

        pilot = new TecbotController(0, TecbotConstants.CONTROLLER_TYPE_PILOT);
        copilot = new TecbotController(1, TecbotConstants.CONTROLLER_TYPE_COPILOT);


        //pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)  ));
        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()  ));



        pilot.whenPressed(TecbotController.ButtonType.POV_UP, new ChassisSetDefaultDrive());

//        pilot.whenPressed(TecbotController.ButtonType.Y, new ShooterGoToTarget());
//        pilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());

        pilot.whenPressed(TecbotController.ButtonType.LB, new ShooterOff());
        pilot.whenPressed(TecbotController.ButtonType.LB, new FeederStop());
        //pilot.whenPressed(TecbotController.ButtonType.RB, new FeederStop());
        pilot.whenPressed(TecbotController.ButtonType.RB, new FeederSetToSpeed());
        pilot.whenPressed(TecbotController.ButtonType.RB, new ShooterGoToTarget());

        pilot.whenPressed(TecbotController.ButtonType.A, new IntakeToggleMotors());


        pilot.whileHeld(TecbotController.ButtonType.X, new RollersRunThenStop() );




        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new DriveTurretManually());

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new MoveTurretToCenter() );


    }

    // singleton
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }

    public double getRollersInput() {
        return pilot.getRightAxisX();
    }

    public TecbotController getPilot() {

        return pilot;
    }

    public TecbotController getCopilot() {

        return copilot;

    }

}
