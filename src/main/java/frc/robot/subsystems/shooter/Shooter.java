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
import frc.robot.resources.TecbotMotorList;

public class Shooter extends SubsystemBase {

    private final TecbotMotorList shooterMotors;

    private SparkMaxPIDController shooterPIDController;
    private RelativeEncoder shooterEncoder;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double shooterPIDTarget;


    public Shooter() {

        shooterMotors = RobotConfigurator.buildMotorList(
                RobotMap.SHOOTER_MOTOR_PORTS,
                RobotMap.SHOOTER_INVERTED_MOTORS,
                RobotMap.SHOOTER_MOTOR_TYPES);

        // shooterEncoder = shooterMotors.getSpecificMotor(RobotMap.SHOOTER_ENCODER_MOTOR_PORT).
        //        getCANSparkMax().getEncoder();

        shooterMotors.getSpecificMotor(RobotMap.SHOOTER_MOTOR_SLAVE_PORT).getCANSparkMax().
                follow(shooterMotors.getSpecificMotor(RobotMap.SHOOTER_MOTOR_MASTER_PORT).getCANSparkMax()
                        , true);

        initPID();

    }

    public void initPID() {

        CANSparkMax masterMotor = shooterMotors.getSpecificMotor(RobotMap.SHOOTER_MOTOR_MASTER_PORT).getCANSparkMax();

        shooterEncoder = masterMotor.getEncoder();
        shooterPIDController = masterMotor.getPIDController();

        kP = RobotMap.SHOOTER_PID_kP;
        kI = RobotMap.SHOOTER_PID_kI;
        kD = RobotMap.SHOOTER_PID_kD;
        kIz = RobotMap.SHOOTER_PID_kIz;
        kFF = RobotMap.SHOOTER_PID_kFF;
        kMaxOutput = RobotMap.SHOOTER_PID_kMaxOutput;
        kMinOutput = RobotMap.SHOOTER_PID_kMinOutput;
        maxRPM = RobotMap.SHOOTER_PID_kMaxRPM;

        shooterPIDTarget = RobotMap.SHOOTER_PID_Target;

        // setShooterPIDController(shooterPIDController);

    }

    public SparkMaxPIDController getShooterPIDController(){
        return shooterPIDController;
    }

    public RelativeEncoder getShooterEncoder() {
        return shooterEncoder;
    }

    public void setShooterPIDController(SparkMaxPIDController pidController) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public REVLibError shooterSetPIDReference(double reference,  SparkMaxPIDController pidController) {
        reference = Math.clamp(reference, -maxRPM, maxRPM);
        return pidController.setReference(reference, CANSparkMax.ControlType.kVelocity);

    }

    public REVLibError shooterSetDefaultPIDReference( SparkMaxPIDController pidController){
        return pidController.setReference(Math.clamp(shooterPIDTarget, -maxRPM, maxRPM) , CANSparkMax.ControlType.kVelocity);
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