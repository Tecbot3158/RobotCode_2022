package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.intake.IntakeEjectThenStop;
import frc.robot.commands.intake.basic.IntakeToggleEject;
import frc.robot.commands.intake.basic.IntakeToggleMotors;
import frc.robot.commands.intake.basic.IntakeTogglePositionAndMotors;
import frc.robot.commands.rollers.RollersRunThenStop;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.turret.DriveTurretToLeft;
import frc.robot.commands.turret.DriveTurretToRight;
import frc.robot.commands.turret.MoveTurretToAngle;
import frc.robot.commands.turret.MoveTurretToCenter;
import frc.robot.resources.Math;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;

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

        pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT,
                new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyRise,
                        Robot.getRobotContainer().getDriveTrain()));
        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyLower,
        // Robot.getRobotContainer().getDriveTrain()));
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
<<<<<<< HEAD
        //
        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new MoveTurretToAngle(30));
=======
//
        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new MoveTurretToAngle(90));
>>>>>>> 53db3e5e41500116d8e02500d373fff08910c6c0

        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new
        // DriveTurretManually());

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // MoveTurretToCenter() );

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
        switch (TecbotConstants.CURRENT_PILOT) {
            case ALEXG:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1);
            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1) * 0.85;
            case PONCE:
                return Math.clamp(-(OI.getInstance().getPilot().getRightAxisX(false)), -1, 1);
            case ESTEBATO:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1);
            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1);

        }
    }

    public double getDefaultDriveInputY() {
        switch (TecbotConstants.CURRENT_PILOT) {
            case ALEXG:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);
            case PAULO:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getTriggers()), -1, 1);
            case PONCE:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);
            case ESTEBATO:
                pilot.setOffset(0.03);
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);
            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);

        }
    }

    // singleton
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }
}
