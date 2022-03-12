package frc.robot;

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

    }


    // singleton
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }
}
