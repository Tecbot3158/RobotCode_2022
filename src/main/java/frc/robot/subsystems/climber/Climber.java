/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotMotorList;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid pistonHanger;
    private final TecbotMotorList ropeController;

    private SparkMaxPIDController ropePIDController0;
    private RelativeEncoder ropeEncoder0;

    private SparkMaxPIDController ropePIDController1;
    private RelativeEncoder ropeEncoder1;

    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
    private double ropePIDController0TargetRPM;


    public Climber() {

        ropeController = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_MOTOR_PORTS, RobotMap.CLIMBER_INVERTED_MOTORS, RobotMap.CLIMBER_MOTOR_TYPES);
        pistonHanger = RobotConfigurator.buildDoubleSolenoid(RobotMap.CLIMBER_SOLENOID_PORTS);


        //TODO disable dragon fly in command for climber! Always!
        //TODO Dragon fly cannot be extended!!

        initPID();

    }

    public void initPID() {
        ropePIDController0 = ropeController.getMotors().get(0).getCANSparkMax().getPIDController();
        ropeEncoder0 = ropeController.getMotors().get(0).getCANSparkMax().getEncoder();

        ropePIDController1 = ropeController.getMotors().get(1).getCANSparkMax().getPIDController();
        ropeEncoder1 = ropeController.getMotors().get(1).getCANSparkMax().getEncoder();

        kP = RobotMap.CLIMBER_PID_kP;
        kI = RobotMap.CLIMBER_PID_kI;
        kD = RobotMap.CLIMBER_PID_kD;
        kIz = RobotMap.CLIMBER_PID_kIz;
        kFF = RobotMap.CLIMBER_PID_kFF;
        kMaxOutput = RobotMap.CLIMBER_PID_kMaxOutput;
        kMinOutput = RobotMap.CLIMBER_PID_kMinOutput;
        maxRPM = RobotMap.CLIMBER_PID_kMaxRPM;

        ropePIDController0TargetRPM = RobotMap.CLIMBER_PID_Target;

        ropeControllerSetPIDValues(ropePIDController0);

        ropeControllerSetPIDValues(ropePIDController1);


    }

    private void ropeControllerSetPIDValues(SparkMaxPIDController pidController) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIz);
        pidController.setFF(kFF);
        pidController.setOutputRange(kMinOutput, kMaxOutput);

    }

    public void runPID(SparkMaxPIDController pidController) {
        double setPoint = ropePIDController0TargetRPM;
        pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

    }

    public void setPistonExtend() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_EXTENDED_POSITION);
    }

    public void setPistonRetract() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_RETRACTED_POSITION);
    }

    public void setPistonRaw(DoubleSolenoid.Value state) {
        pistonHanger.set(state);
    }

    public void setRopeControllerRaw(double speed) {
        ropeController.setAll(speed);
    }

    public TecbotMotorList getRopeControllerList() {
        return ropeController;
    }

    /**
     * Returns a CAN Spark Max {@link RelativeEncoder}
     *
     * @param index index of the MotorList to retrieve the encoder from.
     */
    public RelativeEncoder getRopeControllerEncoder(int index) {
        return ropeController.getMotors().get(index).getCANSparkMax().getEncoder();
    }


}