package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.OI;

import java.util.function.BooleanSupplier;

public class TecbotControllerLeftTriggerButton extends Button implements BooleanSupplier {

    public TecbotControllerLeftTriggerButton() {

    }

    @Override
    public boolean getAsBoolean() {
        // return OI.getInstance().getCopilot().getLeftTrigger() > 0.25;
        return OI.getInstance().getCopilot().getRawAxis(2, false) > 0.25;
    }
}
