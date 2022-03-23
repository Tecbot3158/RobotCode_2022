// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.resources;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.resources.Math;
import frc.robot.subsystems.vision.TecbotCamera;
import org.photonvision.PhotonCamera;

public class VisionValueNormalizer {

    PhotonCamera photonCamera;
    TecbotCamera tecbotCamera;

    int count = 0;
    int maxCount = 20;

    double sum = 0;
    double averageArea = 0;

    double minimumArea;
    double maximumArea;
    double range;

    /**
     * Creates a new Tecbot camera.
     * <p></p>
     * It uses Photonvision with PhotonLib to retrieve the values
     * from the Network Tables.
     * <p>
     * It allows to easily retrieve target values ranging from <code>-1 to 1</code>
     */
    public VisionValueNormalizer(double minArea, double maxArea) {

        this.minimumArea = minArea;
        this.maximumArea = maxArea;

        this.range = this.maximumArea - this.minimumArea;

        tecbotCamera = Robot.getRobotContainer().getTecbotCamera();
        photonCamera = tecbotCamera.getPhotonCamera();

    }

    public void executeAverageArea() {
        if (count == maxCount) {
            averageArea = sum / count;
            count = 0;
            sum = 0;
            return;
        }

        ++count;

        tecbotCamera.update();
        double area = tecbotCamera.getArea();
        sum += area;
        SmartDashboard.putNumber("LIME: area ", area);


    }

    public double getAverageArea() {

        return averageArea;

    }

    /**
     * @return average area from 0 to 1.
     */
    public double getNormalizedDistance() {
        double actualAreaDelta = averageArea - minimumArea;
        return 1 - Math.clamp( actualAreaDelta / range, 0, 1) ;

    }


}