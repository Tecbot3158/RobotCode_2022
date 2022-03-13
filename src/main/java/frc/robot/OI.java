package frc.robot;

import frc.robot.commands.feeder.FeederSetToSpeed;
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

        pilot.whileHeld(TecbotController.ButtonType.RB, new FeederSetToSpeed());

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
