// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class TecbotPWMLEDStrip {

    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    int rainbowFirstPixelHue;

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
        rainbowFirstPixelHue = 0;
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

    private void setSequenceRainbow() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
        led.setData(ledBuffer);
    }

    public void setClear() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
        // led.stop();
    }
}