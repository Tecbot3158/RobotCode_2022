package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OI;

import java.util.function.BooleanSupplier;

public class TecbotControllerRightTriggerButton extends Trigger {

    public TecbotControllerRightTriggerButton(){

    }

    @Override
    public boolean get() {
        return OI.getInstance().getCopilot().getRightTrigger() > 0.13;
    }
}
