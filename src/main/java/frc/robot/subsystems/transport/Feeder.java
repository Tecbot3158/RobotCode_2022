// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.transport;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.TecbotSpeedController;

public class Feeder extends SubsystemBase {

    TecbotSpeedController feeder;


    private RelativeEncoder feederEncoder;

    boolean isReady = false;

    public boolean isReady() {
        return isReady;
    }

    public void setReady(boolean ready) {
        isReady = ready;
    }

    //    public static FeederAndShooterState state = FeederAndShooterState.BOTH_OFF;


    /**
     * Creates a new Feeder.
     */
    public Feeder() {
        feeder = new TecbotSpeedController(RobotMap.TRANSPORT_FEEDER_PORT, RobotMap.TRANSPORT_FEEDER_MOTOR_TYPE);
        feeder.setInverted(RobotMap.TRANSPORT_FEEDER_IS_INVERTED);

        feederEncoder = feeder.getCANSparkMax().getEncoder();

    }

    public TecbotSpeedController getFeederMotor() {
        return feeder;
    }

    public void setRaw(double speed) {
        feeder.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public RelativeEncoder getFeederEncoder() {
        return feederEncoder;
    }
}
