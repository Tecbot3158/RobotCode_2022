package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.chassis.drivingModes.ChassisSetDefaultDrive;
import frc.robot.commands.chassis.drivingModes.ToggleMecanum;
import frc.robot.commands.chassis.drivingModes.ToggleSwerve;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.oitests.OITestTriggers;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;
import frc.robot.resources.TecbotController.ButtonType;

public class OI {

    public static OI instance;

    private TecbotController pilot, copilot;
    Joystick a;

    private OI() {

        pilot = new TecbotController(0, TecbotController.TypeOfController.XBOX);
        copilot = new TecbotController(1, TecbotController.TypeOfController.PS4);
        a = new Joystick(3);

    }

    /**
     * must always be called after ALL subsystems are
     * created.
     */
    public void configureButtonBindings() {

        pilot = new TecbotController(0, TecbotConstants.CONTROLLER_TYPE_PILOT);
        copilot = new TecbotController(1, TecbotConstants.CONTROLLER_TYPE_COPILOT);

        // pilot.whileHeld(TecbotController.ButtonType.RB, new FeederSetToSpeed());

        // pilot.whenPressed(TecbotController.ButtonType.A, new OITestTriggers());

        pilot.whenPressed(ButtonType.A, new ChassisSetDefaultDrive());
        pilot.whenPressed(ButtonType.X, new ToggleMecanum());
        pilot.whenPressed(ButtonType.B, new ToggleSwerve());
        // pilot.whenPressed(new Intake);

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
