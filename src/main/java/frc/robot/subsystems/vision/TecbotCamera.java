// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class TecbotCamera extends SubsystemBase {

    PhotonCamera photonCamera;
    PhotonPipelineResult latestResult, previousResult;

    boolean hasTargets = false;

    /**
     * Creates a new Tecbot camera.
     * For now it uses Photonvision with Photonlib to retrieve the values
     * from the Network Tables
     */
    public TecbotCamera() {

        photonCamera = new PhotonCamera(RobotMap.TECBOT_CAMERA_NAME);

    }

    /**
     * Returns yaw value from latest result.
     * <p>
     * </p>
     * <b>{@link TecbotCamera#update() } MUST be called
     * to update the latest Result. Otherwise, the last fetched
     * result will be used.</b>
     *
     * @return the yaw from the best target, if any.
     *         otherwise, returns 0.
     */
    public double getYaw() {
        double yaw = 0;

        if (hasTargets) {
            yaw = latestResult.getBestTarget().getYaw();
        }

        return yaw;

    }

    /**
     * Returns area value from target.
     * <p>
     * </p>
     * <b>{@link TecbotCamera#update() } MUST be called
     * to update the latest Result. Otherwise, the last fetched
     * result will be used.</b>
     *
     * @return the area from the best target, if any.
     *         otherwise, returns 0.
     */
    public double getArea() {
        double area = 0;

        if (hasTargets) {
            area = latestResult.getBestTarget().getArea();
        }

        return area;

    }

    /**
     * Returns pitch value from latest result.
     * <p>
     * </p>
     * <b>{@link TecbotCamera#update() } MUST be called
     * to update the latest Result. Otherwise, the last fetched
     * result will be used.</b>
     *
     * @return the pitch from the best target, if any.
     *         otherwise, returns 0.
     */
    public double getPitch() {
        double pitch = 0;

        if (hasTargets) {
            pitch = latestResult.getBestTarget().getPitch();
        }

        return pitch;

    }

    /**
     * Fetches latest result.
     * It will overwrite {@link TecbotCamera#latestResult} and
     * {@link TecbotCamera#previousResult}
     */
    public void update() {

        previousResult = latestResult;

        latestResult = photonCamera.getLatestResult();

        hasTargets = latestResult.hasTargets();

    }

    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

    public PhotonPipelineResult getPreviousResult() {
        return previousResult;
    }

}