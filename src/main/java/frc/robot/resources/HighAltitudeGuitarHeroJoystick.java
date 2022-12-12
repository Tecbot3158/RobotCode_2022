// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class HighAltitudeGuitarHeroJoystick {
    Joystick joystick;

    public enum DriveLayout {
        ALEX_S
    }

    public enum ButtonType {
        GREEN,
        RED,
        YELLOW,
        BLUE,
        ORANGE,
        COMBO,
        BACK,
        START,
        POV_N,
        POV_NE,
        POV_E,
        POV_SE,
        POV_S,
        POV_SW,
        POV_W,
        POV_NW,
        VOLUME_1,
        VOLUME_2
    }

    public enum AxisType {
        PICKUP_SWITCH,
        VOLUME_RAMP
    }

    private HashMap<Integer, JoystickButton> availableJoystickButtons;
    private HashMap<Integer, POVButton> availablePOVButtons;
    private HashMap<Integer, Button> availableAxisButtons;

    private HashMap<AxisType, Integer> axisConfiguration;
    private HashMap<AxisType, Double> axisDeadzoneConfiguration;
    private HashMap<AxisType, Double> axisMultiplierConfiguration;

    private HashMap<ButtonType, Button> joystickButtonConfiguration;

    DriveLayout currentDriveLayout;

    public HighAltitudeGuitarHeroJoystick(int port) {
        joystick = new Joystick(port);
        configureGuitar();
        configureDefaultDeadzoneAndMultiplier(0, 1);

        currentDriveLayout = DriveLayout.ALEX_S;
    }

    void configureGuitar() {
        availableJoystickButtons = new HashMap<Integer, JoystickButton>();
        for (int i = 1; i <= joystick.getButtonCount(); i++) {
            availableJoystickButtons.put(i, new JoystickButton(joystick, i));
        }

        availablePOVButtons = new HashMap<Integer, POVButton>();
        for (int i = 0; i <= 360; i += 45) {
            availablePOVButtons.put(i, new POVButton(joystick, i));
        }

        availableAxisButtons = new HashMap<Integer, Button>();
        for (int i = 0; i < 6; i++) {
            int currentPort = i;
            BooleanSupplier booleanSupplier = () -> isAxisPressed(currentPort);
            availableAxisButtons.put(i, new Button(booleanSupplier));
        }

        axisConfiguration = new HashMap<AxisType, Integer>();
        axisConfiguration.put(AxisType.PICKUP_SWITCH, 2);
        axisConfiguration.put(AxisType.VOLUME_RAMP, 4);

        joystickButtonConfiguration = new HashMap<ButtonType, Button>();
        joystickButtonConfiguration.put(ButtonType.GREEN, availableJoystickButtons.get(1));
        joystickButtonConfiguration.put(ButtonType.RED, availableJoystickButtons.get(2));
        joystickButtonConfiguration.put(ButtonType.YELLOW, availableJoystickButtons.get(4));
        joystickButtonConfiguration.put(ButtonType.BLUE, availableJoystickButtons.get(3));
        joystickButtonConfiguration.put(ButtonType.ORANGE, availableJoystickButtons.get(5));
        joystickButtonConfiguration.put(ButtonType.BACK, availableJoystickButtons.get(7));
        joystickButtonConfiguration.put(ButtonType.START, availableJoystickButtons.get(8));
        joystickButtonConfiguration.put(ButtonType.COMBO, availableJoystickButtons.get(9));

        joystickButtonConfiguration.put(ButtonType.POV_N, availablePOVButtons.get(0));
        joystickButtonConfiguration.put(ButtonType.POV_NE, availablePOVButtons.get(45));
        joystickButtonConfiguration.put(ButtonType.POV_E, availablePOVButtons.get(90));
        joystickButtonConfiguration.put(ButtonType.POV_SE, availablePOVButtons.get(135));
        joystickButtonConfiguration.put(ButtonType.POV_S, availablePOVButtons.get(180));
        joystickButtonConfiguration.put(ButtonType.POV_SW, availablePOVButtons.get(225));
        joystickButtonConfiguration.put(ButtonType.POV_W, availablePOVButtons.get(270));
        joystickButtonConfiguration.put(ButtonType.POV_NW, availablePOVButtons.get(315));
    }

    private void configureDefaultDeadzoneAndMultiplier(double deadzone, double multiplier) {
        axisDeadzoneConfiguration = new HashMap<AxisType, Double>();
        axisMultiplierConfiguration = new HashMap<AxisType, Double>();
        for (AxisType axisType : AxisType.values()) {
            axisDeadzoneConfiguration.put(axisType, deadzone);
            axisMultiplierConfiguration.put(axisType, multiplier);
        }
    }

    public double getRawAxis(AxisType axisType) {
        return joystick.getRawAxis(axisConfiguration.get(axisType));
    }

    public double getRawAxis(int axisPort) {
        return joystick.getRawAxis(axisPort);
    }

    public double getAxis(AxisType axis) {
        double input = getRawAxis(axis);
        double clampedInput = clampToDeadzone(input, axisDeadzoneConfiguration.get(axis));
        double multipliedInput = clampedInput * axisMultiplierConfiguration.get(axis);
        return multipliedInput;
    }

    public boolean isAxisPressed(AxisType axisType) {
        return Math.abs(getAxis(axisType)) > 0.5;
    }

    public boolean isAxisPressed(int axisPort) {
        return Math.abs(getRawAxis(axisPort)) > 0.5;
    }

    private double clampToDeadzone(double input, double deadzone) {
        return Math.abs(input) > deadzone ? input : 0;
    }

    public void setAxisDeadzone(AxisType axis, double deadzone) {
        axisDeadzoneConfiguration.put(axis, Math.abs(deadzone));
    }

    public void setAxisMultiplier(AxisType axis, double multiplier) {
        axisMultiplierConfiguration.put(axis, multiplier);
    }

    public Button getButtonObj(ButtonType buttonType) {
        return joystickButtonConfiguration.get(buttonType);
    }

    public JoystickButton getJoystickButtonObj(int port) {
        return availableJoystickButtons.get(port);
    }

    public Button getPOVButtonObj(int angle) {
        return availablePOVButtons.get(angle);
    }

    public Button getAxisButtonObj(int axis) {
        return availableAxisButtons.get(axis);
    }

    // METHODS FOR ASSOCIATING COMMANDS YES THEY'RE A LOT BUT WE'D RATHER HAVE IT
    // THIS WAY

    /**
     * Set commands to be executed when a certain button is pressed. The command
     * will schedule once when the button's pressed, but the command will not be
     * interrupted if it's released.
     *
     * @param buttonType this is the button which will be assigned the command
     * @param command    command to be assigned to button
     */
    public void whenPressed(ButtonType buttonType, CommandBase command) {
        Button chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whenPressed(command);
        else
            DriverStation.reportWarning("Button not found!", true);
    }

    /**
     * Set commands to be executed continuously while a certain button is held.
     * The command will be constantly scheduled while it's held.
     *
     * @param buttonType this is the button which will be assigned the command
     * @param command    command to be assigned to button
     */
    public void whileHeld(ButtonType buttonType, CommandBase command) {
        Button chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whileHeld(command);
    }

    /**
     * Set commands to be executed when a certain button is held.
     * The command will be constantly scheduled while it's held.
     *
     * @param buttonType this is the button which will be assigned the command
     * @param command    command to be assigned to button
     */
    public void whenHeld(ButtonType buttonType, CommandBase command) {
        Button chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whenHeld(command);
    }

    /**
     * Set commands to be executed when a certain button is held.
     * The command will be constantly scheduled while it's held.
     *
     * @param buttonType this is the button which will be assigned the command
     * @param command    command to be assigned to button
     */
    public void whenReleased(ButtonType buttonType, CommandBase command) {
        Button chosenButton = joystickButtonConfiguration.get(buttonType);
        if (chosenButton != null)
            chosenButton.whenReleased(command);
    }

    /**
     * Set commands to be executed when a series of buttons are pressed. The command
     * will schedule once when the buttons are pressed, but the command will not be
     * interrupted if they're released.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will be assigned the command
     */
    public void whenPressedCombo(CommandBase command, ButtonType... buttons) {
        Trigger triggerList = joystickButtonConfiguration.get(buttons[0]);
        for (int i = 1; i < buttons.length; i++) {
            Button chosenButton = joystickButtonConfiguration.get(buttons[i]);
            if (chosenButton != null)
                triggerList = triggerList.and(chosenButton);
            else
                DriverStation.reportWarning("Button " + buttons[i] + " not found!", true);
        }
        Button buttonList = new Button(triggerList);

        buttonList.whenPressed(command);
    }

    /**
     * Set commands to be executed continuously while certain buttons are held.
     * The command will be constantly scheduled while they're held.
     *
     * @param command command to be assigned to button
     * @param buttons these are the buttons which will be assigned the command
     */
    public void whileHeldCombo(CommandBase command, ButtonType... buttons) {
        Trigger triggerList = joystickButtonConfiguration.get(buttons[0]);
        for (int i = 1; i < buttons.length; i++) {
            Button chosenButton = joystickButtonConfiguration.get(buttons[i]);
            if (chosenButton != null)
                triggerList = triggerList.and(chosenButton);
            else
                DriverStation.reportWarning("Button " + buttons[i] + " not found!", true);
        }
        Button buttonList = new Button(triggerList);

        buttonList.whileHeld(command);
    }

    public double getDriveX() {
        switch (currentDriveLayout) {
            case ALEX_S:
                double speed = getAxis(AxisType.PICKUP_SWITCH);
                double direction;
                if (getButtonObj(ButtonType.GREEN).get())
                    direction = -1;
                else if (getButtonObj(ButtonType.YELLOW).get())
                    direction = 1;
                else
                    direction = 0;

                return speed * direction;
            default:
                return 0;
        }
    }

    public double getDriveY() {
        switch (currentDriveLayout) {
            case ALEX_S:
                double speed = getAxis(AxisType.PICKUP_SWITCH);
                double direction;
                if (getButtonObj(ButtonType.POV_N).get())
                    direction = 1;
                else if (getButtonObj(ButtonType.POV_S).get())
                    direction = -1;
                else
                    direction = 0;

                return speed * direction;
            default:
                return 0;
        }
    }

}
