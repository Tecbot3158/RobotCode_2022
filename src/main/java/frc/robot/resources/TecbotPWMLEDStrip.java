// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.resources;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class TecbotPWMLEDStrip {

    AddressableLED led;
    AddressableLEDBuffer ledBuffer;

    // FOR RAINBOW AND SOLID CYCLE
    int firstPixelData;

    // FOR FIRE ANIMATION
    int cooling, sparking, cooldown, heat[];

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

        // FOR RAINBOW AND SOLID CYCLE
        firstPixelData = 0;

        // FOR FIRE ANIMATION
        heat = new int[length];
        for (int i = 0; i < heat.length; i++) {
            heat[i] = 255;
        }

    }

    /**
     * Sets a {@link TecbotLEDStrip} to a single solid color.
     * Please make sure that the parameters are in the following range:
     * 
     * @param hue        The HUE of the color 0-180
     * @param saturation The SATURATION of the color 0-255
     * @param value      The VALUE of the color 0-255
     */
    public void setSolidHSV(int hue, int saturation, int value) {

        hue = (int) Math.clamp(hue, 0, 180);
        saturation = (int) Math.clamp(saturation, 0, 255);
        value = (int) Math.clamp(value, 0, 255);

        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, hue, saturation, value);
        }
        led.setData(ledBuffer);
    }

    public void setRainbowCycle() {
        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (firstPixelData + (i * 180 / ledBuffer.getLength())) % 180;
            // Set the value
            ledBuffer.setHSV(i, hue, 255, 255);
        }
        // Increase by to make the rainbow "move"
        firstPixelData += 3;
        // Check bounds
        firstPixelData %= 180;
        led.setData(ledBuffer);
    }

    public void setSolidCycle(int hue, int saturation) {

        hue = (int) Math.clamp(hue, 0, 180);
        saturation = (int) Math.clamp(saturation, 0, 255);

        // For every pixel
        for (var i = 0; i < ledBuffer.getLength(); i++) {
            final var value = (firstPixelData + (i * 180 / ledBuffer.getLength())) % 180;
            ledBuffer.setHSV(i, hue, saturation, value);
        }
        firstPixelData += 5;
        // Check bounds
        firstPixelData %= 180;
        led.setData(ledBuffer);
    }

    public void setAmber() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, 20, 237, 255);
        }
        led.setData(ledBuffer);
    }

    public void setClear() {
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setHSV(i, 0, 0, 0);
        }
        led.setData(ledBuffer);
        // led.stop();
    }

    /**
     * Starts a fire animation of a color given its hue and saturation.
     * 
     * @param hue        hue of the color
     * @param saturation saturation of the color
     * @param cooling    How much the fire cools down. The lower the value, the more
     *                   intense the fire is. Recommended values: 10 when idle, 0
     *                   when moving.
     * @param sparking   How many sparks are randomly generated. The larger the
     *                   value, the more sparks are created. Recommended values: 10
     *                   when idle, 210 when moving.
     */
    public void setFire(int hue, int saturation, int cooling, int sparking) {
        // Step 0. Check for mistakes from 8th layer and ensure parameters are alright
        hue = (int) Math.clamp(hue, 0, 180);
        saturation = (int) Math.clamp(saturation, 0, 255);
        cooling = (int) Math.clamp(cooling, 0, 255);
        sparking = (int) Math.clamp(sparking, 0, 255);

        // Step 1. Cool down every cell a little
        for (int i = 0; i < ledBuffer.getLength(); i++) {
            cooldown = Math.randomInt(0, ((cooling * 10) / ledBuffer.getLength()) + 2);
            if (cooldown > heat[i] || heat[i] > 255) {
                heat[i] %= 255;
            } else {
                heat[i] -= cooldown;
            }
        }

        // Step 2. Heat from each cell drifts 'up' and diffuses a little
        for (int k = ledBuffer.getLength() - 1; k >= 2; k--) {
            heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
        }

        heat[1] = (heat[1] + 2 * heat[0]) / 3;

        // Step 3. Randomly ignite new 'sparks' near the bottom
        if (Math.randomInt(0, 255) < sparking) {
            int y = Math.randomInt(0, 7);

            heat[y] += Math.randomInt(160, 255);
        }

        // Step 4. Convert heat to LED colors
        for (int j = 0; j < ledBuffer.getLength(); j++) {
            ledBuffer.setHSV(j, hue, saturation, heat[j]);
        }

        led.setData(ledBuffer);
    }
}