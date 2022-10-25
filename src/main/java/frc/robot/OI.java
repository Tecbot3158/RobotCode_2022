package frc.robot;

import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.chassis.drivingModes.ChassisToggleTransmissionMode;
import frc.robot.commands.chassis.drivingModes.ToggleDragonFlyRaw;
import frc.robot.commands.chassis.drivingModes.ToggleDragonFlyWheel;
import frc.robot.commands.chassis.drivingModes.ToggleInputState;
//import frc.robot.commands.chassis.drivingModes.ToggleDragonFlyWheel;
import frc.robot.commands.climber.ClimberSetInitThenClearBindingsThenSetClimberBindings;
import frc.robot.commands.climber.ClimberToggleFreeMode;
import frc.robot.commands.climber.basic.ClimberSetRawMotors;
import frc.robot.commands.climber.basic.ClimberToggleSolenoids;
import frc.robot.commands.feeder.FeederEjectThenStop;
import frc.robot.commands.feeder.FeederSetToSpeedThenStop;
import frc.robot.commands.feeder.FeederStop;
import frc.robot.commands.intake.IntakeTogglePositionAndMotors;
import frc.robot.commands.intake.basic.IntakeToggleEject;
import frc.robot.commands.intake.basic.IntakeToggleMotors;
import frc.robot.commands.rollers.RollersRunThenStop;
import frc.robot.commands.shooter.ShooterGoToTarget;
import frc.robot.commands.shooter.ShooterGoToVariableVisionTarget;
import frc.robot.commands.shooter.ShooterGoToVeryHighValue;
import frc.robot.commands.shooter.ShooterShootCustomTarger;
import frc.robot.commands.shooter.basic.ShooterGoRaw;
import frc.robot.commands.shooter.basic.ShooterOff;
import frc.robot.commands.turret.DriveTurretToAngle;
import frc.robot.commands.turret.basic.DriveTurretToLeft;
import frc.robot.commands.turret.basic.DriveTurretToRight;
import frc.robot.resources.Math;
import frc.robot.resources.*;

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

        // pilot.whenPressed(TecbotController.ButtonType.POV_UP, new
        // ChassisSetDefaultDrive());
        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
        // Robot.getRobotContainer().getDriveTrain()::setMecanumDrive,
        // Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)
        // ));

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
        // Robot.getRobotContainer().getDriveTrain()::setMecanumDrive,
        // Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new
        // ToggleDragonFlyWheel());

        pilot.whenPressed(TecbotController.ButtonType.LB, new ChassisToggleTransmissionMode());
        pilot.whenPressed(TecbotController.ButtonType.RB, new ShooterGoToVeryHighValue());

        pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new ToggleDragonFlyRaw());
        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyLower,y
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

        pilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new ToggleInputState());

        // pilot.whileHeld(TecbotController.ButtonType.Y, new IntakeEjectThenStop());

        // COPILOT:

        copilot.whileHeld(TecbotController.ButtonType.RB, new DriveTurretToRight());
        copilot.whileHeld(TecbotController.ButtonType.LB, new DriveTurretToLeft());

        // pilot.whenPressed(TecbotController.ButtonType.B, new IntakeToggle());

        copilot.whileHeld(TecbotController.ButtonType.X, new RollersRunThenStop());
        copilot.whileHeld(TecbotController.ButtonType.X, new FeederSetToSpeedThenStop());

        // copilot.whenPressed(TecbotController.ButtonType.A, new ShooterGoToTarget());
        // copilot.whenPressed(TecbotController.ButtonType.A, new
        // ShooterGoToVariableVisionTarget());

        // copilot.whenPressed(TecbotController.ButtonType.A, new );
        // copilot.whenPressed(TecbotController.ButtonType.A, new
        // ShooterShootCustomTarger(1500));
        copilot.whenPressed(TecbotController.ButtonType.A, new ShooterGoToTarget());
        // copilot.whenPressed(TecbotController.ButtonType.A, new FeederSetToSpeed());

        copilot.whenPressed(TecbotController.ButtonType.B, new ShooterOff());
        copilot.whenPressed(TecbotController.ButtonType.B, new FeederStop());

        copilot.whileHeld(TecbotController.ButtonType.Y, new FeederEjectThenStop());

        /*
         * The following turret angles are relative to the 'opposite of the robot.
         * E.g. the direction of the intake would be 180 degrees,
         * and it goes in the positive direction when going counterclockwise.
         * 
         * That is, the zero deg ( 0 deg ) would be opposite of the intake,
         * the left of the intake would be 270 degrees,
         * 90 degrees would be to the right of the intake, etc.
         * 
         * ~~(180)~~
         * | |
         * | |
         * (270) | | (90)
         * | |
         * | |
         * ___(0)___
         * 
         * 
         */

        copilot.whenPressed(TecbotController.ButtonType.POV_DOWN, new DriveTurretToAngle(0));
        copilot.whenPressed(TecbotController.ButtonType.POV_UP, new DriveTurretToAngle(184));
        copilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new DriveTurretToAngle(90));
        copilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new DriveTurretToAngle(270));

        new TecbotControllerLeftTriggerButton().whileHeld(new ShooterGoRaw(0.4));

        // new TecbotControllerLeftTriggerButton().whileHeld(new
        // ShooterShootCustomTarger(1500));
        // Maximum5600
        // 0.3-1600-1700
        // 0.27-1500 revoluciones
        // new TecbotControllerRightTriggerButton().whileActive(new ShooterGoRaw(0.3));

        // copilot.whenPressed(TecbotController.ButtonType.BACK, new
        // ClimberInitiateClimbingMode());
        // Updated upstream
        // copilot.whenPressed(TecbotController.ButtonType.START,
        // new ClimberSetInitThenClearBindingsThenSetClimberBindings());
        // copilot.getRawButtonObj(5).whenPressed(new
        // ClimberSetInitThenClearBindingsThenSetClimberBindings());
        // copilot.whenPressed(TecbotController.ButtonType.BACK,
        // new ClimberSetInitThenClearBindingsThenSetClimberBindings());
        // copilot.whenPressed(TecbotController.ButtonType.LS, new ShooterGoRaw(0.3));
        // copilotBackButton.whenPressed(new
        // ClimberSetInitThenClearBindingsThenSetClimberBindings());
        // copilot.whenPressed(TecbotController.ButtonType.START, new
        // ClimberToggleSolenoids());
        copilot.whenPressed(TecbotController.ButtonType.BACK,
                new ClimberSetInitThenClearBindingsThenSetClimberBindings());
        // copilot.whenPressed(TecbotController.ButtonType.LS, new ShooterGoRaw(0.4));

        // copilot.whenPressed(TecbotController.ButtonType.BACK,
        // new ClimberSetInitThenClearBindingsThenSetClimberBindings());
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
                return Math.clamp(-(OI.getInstance().getPilot().getRightAxisX(false)), -1, 1) * 0.6;

            case PAULO:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1) * 0.6;

            case ALEXS235:
            case ALEXG:
            case ESTEBATO:
            case JOAQUIN:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisX(false)), -1, 1) * 0.7;

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
            case JOAQUIN:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);

            default:
                return Math.clamp(-(OI.getInstance().getPilot().getLeftAxisY(false)), -1, 1);

        }
    }

    public double getMiddleWheel() {
        return pilot.getRightAxisX();
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
     * Intended for use <b>only</b> in the {@link ClimberSetRawMotors}
     * command.
     * </p>
     * <p>
     *
     * @return speed ranging from -1 to 1. Inclusive on both sides.
     *         </p>
     */
    public double getClimberDefaultManualInput() {
        return -copilot.getLeftAxisY();
    }

    public void configureButtonBindingsForClimbingMode() {

        pilot.whenPressed(TecbotController.ButtonType.POV_UP, new ChassisSetDefaultDrive());
        pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
                Robot.getRobotContainer().getDriveTrain()::setMecanumDrive, Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain().setDrivingMode(DriveTrain.DrivingMode.Mecanum)
        // ));

        // pilot.whenPressed(TecbotController.ButtonType.POV_LEFT, new InstantCommand(
        // Robot.getRobotContainer().getDriveTrain()::setMecanumDrive,
        // Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT,
        // new
        // InstantCommand(Robot.getRobotContainer().getDriveTrain()::setDragonFlyRise,
        // Robot.getRobotContainer().getDriveTrain()));

        // pilot.whenPressed(TecbotController.ButtonType.POV_RIGHT, new
        // ToggleDragonFlyWheel());

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

        copilot.whenPressed(TecbotController.ButtonType.START, new ClimberToggleSolenoids());
        // copilot.whenPressed(TecbotController.ButtonType.BACK, new
        // ClimberInitiateClimbingMode());

        // RS toggle climb free and non-free mode.

        copilot.whenPressed(TecbotController.ButtonType.RS, new ClimberToggleFreeMode());

    }

}

// axis para rueda ???