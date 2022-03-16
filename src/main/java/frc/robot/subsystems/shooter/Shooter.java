/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.shooter;


import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.StepControl;
import frc.robot.resources.TecbotMotorList;

public class Shooter extends SubsystemBase {

    private final TecbotMotorList shooterMotors;

    private RelativeEncoder shooterEncoder;

    StepControl stepControl;


    public Shooter() {

        // stepControl = new StepControl();

        shooterMotors = RobotConfigurator.buildMotorList(
                RobotMap.SHOOTER_MOTOR_PORTS,
                RobotMap.SHOOTER_INVERTED_MOTORS,
                RobotMap.SHOOTER_MOTOR_TYPES);

        // shooterEncoder = shooterMotors.getSpecificMotor(RobotMap.SHOOTER_ENCODER_MOTOR_PORT).
        //        getCANSparkMax().getEncoder();

        shooterMotors.getSpecificMotor(RobotMap.SHOOTER_MOTOR_SLAVE_PORT).getCANSparkMax().
                follow(shooterMotors.getSpecificMotor(RobotMap.SHOOTER_MOTOR_MASTER_PORT).getCANSparkMax()
                        , true);


    }

    public RelativeEncoder getShooterEncoder() {
        return shooterEncoder;
    }

    /**
     * Returns a CAN Spark Max {@link RelativeEncoder}
     *
     * @param index index of the MotorList to retrieve the encoder from.
     */
    public RelativeEncoder getShooterEncoderByIndex(int index) {
        return shooterMotors.getMotors().get(index).getCANSparkMax().getEncoder();
    }

    public RelativeEncoder getShooterEncoderByPort(int port) {
        return shooterMotors.getSpecificMotor(port).getCANSparkMax().getEncoder();
    }


}