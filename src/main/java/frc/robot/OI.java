package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.feeder.FeederSetToSpeed;
import frc.robot.commands.oitests.OITestTriggers;
import frc.robot.resources.TecbotConstants;
import frc.robot.resources.TecbotController;

public class OI {

    public static OI instance;

    private TecbotController pilot, copilot;

    private OI() {

        pilot = new TecbotController(0, TecbotController.TypeOfController.XBOX);
        copilot = new TecbotController(1, TecbotController.TypeOfController.PS4);

    }

    /**
     * must always be called after ALL subsystems are
     * created.
     */
    public void configureButtonBindings() {

        pilot = new TecbotController(0, TecbotConstants.CONTROLLER_TYPE_PILOT);
        copilot = new TecbotController(1, TecbotConstants.CONTROLLER_TYPE_COPILOT);

        pilot.whileHeld(TecbotController.ButtonType.POV_0, new OITestTriggers());

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
