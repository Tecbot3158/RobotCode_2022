// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class TecbotPWMLEDStrip {

    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    /**
     * Creates a new Addressable LED strip connected through PWM.
     * 
     * @param port   PWM port for the strip
     * @param length Amount of LEDs in the strip
     */
    public TecbotPWMLEDStrip(int port, int length) {
        led = new AddressableLED(port);
        led.setLength(length);
        ledBuffer = new AddressableLEDBuffer(length);

        led.setData(ledBuffer);
        led.start();
    }

    /**
     * Sets a {@link TecbotLEDStrip} to a single solid color.
     * Please make sure that the parameters are in the following range:
     * 
     * @param h The HUE of the color 0-180
     * @param s The SATURATION of the color 0-255
     * @param v The VALUE of the color 0-255
     */
    public void setSolidHSV(int h, int s, int v) {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, h, s, v);
        }
        led.setData(ledBuffer);
    }
}