package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.chassis.drivingModes.ChassisToggleTransmissionMode;
import frc.robot.commands.feeder.FeederEjectThenStop;
import frc.robot.commands.feeder.FeederSetToSpeedThenStop;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.intake.basic.IntakeToggleEject;
import frc.robot.commands.intake.basic.IntakeToggleMotors;
import frc.robot.commands.intake.IntakeTogglePositionAndMotors;
import frc.robot.commands.rollers.RollersRunThenStop;
import frc.robot.commands.shooter.ShooterGoToLower;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.shooter.ShooterGoToVeryHighValue;
import frc.robot.commands.shooter.ShooterOff;
import frc.robot.commands.turret.*;
import frc.robot.commands.turret.basic.DriveTurretToLeft;
import frc.robot.commands.turret.basic.DriveTurretToRight;
import frc.robot.resources.*;
import frc.robot.resources.Math;

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

        pilot.whenPressed(TecbotController.ButtonType.POV_UP, new ChassisSetDefaultDrive());
        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
                Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)
        // ));

//        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
//                Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()));

        pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT,
                new InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyRise,
                        Robot.getRobotContainer().getDriveTrain()));

        pilot.whenPressed(TecbotController.ButtonType.LB, new ChassisToggleTransmissionMode());
        pilot.whenPressed(TecbotController.ButtonType.RB, new ShooterGoToVeryHighValue());

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyLower,
        // Robot.getRobotContainer().getDriveTrain()));
        // lower llanta abajo
        // rise llanta arriba


        // pilot.whenPressed(TecbotController.ButtonType.Y, new ShooterGoToTarget());
        // pilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());

        // pilot.whenPressed(TecbotController.ButtonType.A, new IntakeToggleMotors());
        // pilot.whenPressed(TecbotController.ButtonType.A, new IntakeToggle());

        pilot.whenPressed(TecbotController.ButtonType.A, new IntakeTogglePositionAndMotors());
        pilot.whenPressed(TecbotController.ButtonType.B, new IntakeToggleMotors());

        pilot.whenPressed(TecbotController.ButtonType.Y, new IntakeToggleEject());
        // pilot.whileHeld(TecbotController.ButtonType.Y, new IntakeEjectThenStop());



        // COPILOT:

        copilot.whileHeld(TecbotController.ButtonType.RB, new DriveTurretToRight());
        copilot.whileHeld(TecbotController.ButtonType.LB, new DriveTurretToLeft());

        // pilot.whenPressed(TecbotController.ButtonType.B, new IntakeToggle());

        copilot.whileHeld(TecbotController.ButtonType.X, new RollersRunThenStop());
        copilot.whileHeld(TecbotController.ButtonType.X, new FeederSetToSpeedThenStop());

        //copilot.whenPressed(TecbotController.ButtonType.A, new ShooterGoToTarget());
        copilot.whenPressed(TecbotController.ButtonType.A, new ShooterGoToTarget());
        // copilot.whenPressed(TecbotController.ButtonType.A, new FeederSetToSpeed());

        copilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());
        copilot.whenPressed(TecbotController.ButtonType.B, new FeederStop());

        copilot.whileHeld(TecbotController.ButtonType.Y, new FeederEjectThenStop());

        // limelight gatillo

        // copilot.whenPressed(TecbotController.ButtonType.LB, new ShooterOff());
        // copilot.whenPressed(TecbotController.ButtonType.LB, new FeederStop());
        // //pilot.whenPressed(TecbotController.ButtonType.RB, new FeederStop());
        // copilot.whenPressed(TecbotController.ButtonType.RB, new FeederSetToSpeed());
        // copilot.whenPressed(TecbotController.ButtonType.RB, new ShooterGoToTarget());
        //

        //copilot.whileHeld(TecbotController.ButtonType.LS, new DriveTurretToVisionTarget());

        // copilot.whenPressed(TecbotController.ButtonType.);
//        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new DriveTurretToAngle(-180));
//        copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new DriveTurretToCenter());
//        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new DriveTurretToAngle(-90));
//        copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new DriveTurretToAngle(-270));
//
        // copilot.whenPressed(TecbotController.ButtonType.RS, new ShooterGoToLower());
        // JoystickButton

       // copilot.whenPressed(TecbotController.ButtonType.POV_UP, new DriveTurretToAngleRelativeToRobot(0));
       // copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new DriveTurretToAngleRelativeToRobot(180));

        copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new DriveTurretToAngle(0));
        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new DriveTurretToAngle(180));

        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new DriveTurretToAngle(90));
        copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new DriveTurretToAngle(270));



        new TecbotControllerLeftTriggerButton().whileHeld( new DriveTurretToVisionTarget() );
        //copilot.whenPressed(TecbotController.ButtonType., new DriveTurretToAngleRelativeToRobot(270));
        new TecbotControllerRightTriggerButton().whenActive( new ShooterGoToLower() );


        //copilot.whenPressed(TecbotController.ButtonType.RS, new DriveTurretToAngleRelativeToRobotWithAxis());



//        copilot.whenPressed(TecbotController.ButtonType.START, new DriveTurretToAngleRelativeToRobot(180));

        // copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // DriveTurretToVisionTarget());

//        copilot.setOffset(0.13);

        // Shooter shooter = Robot.getRobotContainer().getShooter();
        //copilot.whenPressed(TecbotController.ButtonType.START, new ShooterSetServoSpeeds());

        /// ----- climber ----

//         copilot.whenPressed(TecbotController.ButtonType.BACK, new
//                 ClimberSetRawMotors());
//        //
//        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new
//                ClimberToggleSolenoids());
        // copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new
        // ClimberToggleSolenoids());

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
                return Math.clamp(-(OI.getInstance().getPilot().getRightAxisX(false)), -1, 1) * 0.7;

            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1) * 0.85;

            case ALEXS235:
            case ALEXG:
            case ESTEBATO:
            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1);
        }
    }

    public double getDefaultDriveInputY() {
        pilot.setOffset(0.03);
        // default offset for everyone.
        switch (TecbotConstants.CURRENT_PILOT) {
            // Alex and Esteban want les mêmes choses.
            // and Ponce.
            case TREVCAN:
            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getTriggers()), -1, 1);

            case ALEXS235:
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


    /**
     * get the Climber default raw input for the motors.
     * <p>
     * Intended for use <b>only</b> in the {@link frc.robot.commands.climber.ClimberSetRawMotors}
     * command.
     * </p>
     * <p>
     *
     * @return speed ranging from -1 to 1. Inclusive on both sides.
     * </p>
     */
    public double getClimberDefaultManualInput() {
        return -copilot.getLeftAxisY();
    }

}

// axis para rueda ???