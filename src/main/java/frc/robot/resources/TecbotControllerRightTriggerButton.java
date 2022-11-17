package frc.robot.resources;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.OI;

import java.util.function.BooleanSupplier;

public class TecbotControllerRightTriggerButton extends Button implements BooleanSupplier {

    public TecbotControllerRightTriggerButton() {

    }

    @Override
    public boolean getAsBoolean() {
        System.out.println();
        return OI.getInstance().getCopilot().getRightTrigger() > 0.13;
    }
}
