// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision.resources;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionValueNormalizer extends SubsystemBase {

    PhotonCamera photonCamera;

    /**
     * Creates a new Tecbot camera.
     * <p></p>
     * It uses Photonvision with PhotonLib to retrieve the values
     * from the Network Tables.
     *
     * It allows to easily retrieve target values ranging from <code>-1 to 1</code>
     */
    public VisionValueNormalizer() {

        photonCamera = Robot.getRobotContainer().getTecbotCamera().getPhotonCamera();




    }

}