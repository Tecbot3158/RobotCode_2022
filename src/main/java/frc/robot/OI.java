package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.climber.ClimberSetRawMotors;
import frc.robot.commands.climber.ClimberToggleSolenoids;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.intake.basic.IntakeToggleEject;
import frc.robot.commands.intake.basic.IntakeToggleMotors;
import frc.robot.commands.intake.basic.IntakeTogglePositionAndMotors;
import frc.robot.commands.rollers.RollersRunThenStop;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.shooter.ShooterSetServoSpeeds;
import frc.robot.commands.turret.*;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;
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


        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)
        // ));
        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
                Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()));


        //pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)  ));
        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()));

        pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyRise, Robot.getRobotContainer().getDriveTrain()));
        //pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyLower, Robot.getRobotContainer().getDriveTrain()));
        // lower llanta abajo
        // rise llanta arriba

        pilot.whenPressed(TecbotController.ButtonType.POV_UP, new ChassisSetDefaultDrive());

        // pilot.whenPressed(TecbotController.ButtonType.Y, new ShooterGoToTarget());
        // pilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());

        // pilot.whenPressed(TecbotController.ButtonType.A, new IntakeToggleMotors());
        // pilot.whenPressed(TecbotController.ButtonType.A, new IntakeToggle());

        pilot.whenPressed(TecbotController.ButtonType.A, new IntakeTogglePositionAndMotors());
        pilot.whenPressed(TecbotController.ButtonType.B, new IntakeToggleMotors());

        pilot.whenPressed(TecbotController.ButtonType.Y, new IntakeToggleEject());
        // pilot.whileHeld(TecbotController.ButtonType.Y, new IntakeEjectThenStop());

        copilot.whileHeld(TecbotController.ButtonType.RB, new DriveTurretToRight());
        copilot.whileHeld(TecbotController.ButtonType.LB, new DriveTurretToLeft());

        // pilot.whenPressed(TecbotController.ButtonType.B, new IntakeToggle());

        copilot.whileHeld(TecbotController.ButtonType.X, new RollersRunThenStop());

        copilot.whenPressed(TecbotController.ButtonType.A, new ShooterGoToTarget());
        copilot.whenPressed(TecbotController.ButtonType.A, new FeederSetToSpeed());

        copilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());
        copilot.whenPressed(TecbotController.ButtonType.B, new FeederStop());


        // copilot.whenPressed(TecbotController.ButtonType.LB, new ShooterOff());
        // copilot.whenPressed(TecbotController.ButtonType.LB, new FeederStop());
        // //pilot.whenPressed(TecbotController.ButtonType.RB, new FeederStop());
        // copilot.whenPressed(TecbotController.ButtonType.RB, new FeederSetToSpeed());
        // copilot.whenPressed(TecbotController.ButtonType.RB, new ShooterGoToTarget());
        //

        // copilot.whenPressed(TecbotController.ButtonType.);
        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new MoveTurretToCenter());
        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new MoveTurretToAngle(90));
        copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new MoveTurretToAngle(180));
        copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new MoveTurretToAngle(-90));

        copilot.whenPressed(TecbotController.ButtonType.BACK, new DriveTurretToVisionTarget());


        // copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new DriveTurretToVisionTarget());

        copilot.setOffset( 0.13);

        // Shooter shooter = Robot.getRobotContainer().getShooter();
        copilot.whenPressed(TecbotController.ButtonType.START, new ShooterSetServoSpeeds());

//        copilot.whenPressed(TecbotController.ButtonType.BACK, new ClimberSetRawMotors());
//
//        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new ClimberToggleSolenoids());
//        copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new ClimberToggleSolenoids());

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

    public double getDefaultDriveInputX() {
        // default offset for everyone.
        pilot.setOffset(0.03);

        switch (TecbotConstants.CURRENT_PILOT) {
            case PONCE:
            case ALEXG:
            case ESTEBATO:
            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1);
            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1) * 0.85;
        }
    }

    public double getDefaultDriveInputY() {
        pilot.setOffset(0.03);
        // default offset for everyone.
        switch (TecbotConstants.CURRENT_PILOT) {
            // Alex and Esteban want les mêmes choses.
            // and Ponce.
            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getTriggers()), -1, 1);
            case ALEXG:
            case ESTEBATO:
            case PONCE:
            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);


        }
    }

    public double getTurretDefaultInput() {
        return 0;
    }

    // singleton

    /**
     * get a static OI instance.
     * <p>
     * If no instance has been generated, it
     * will generate one.
     */

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }


    public double getClimberDefaultManualInput(){
        return - copilot.getLeftAxisY();
    }


}
