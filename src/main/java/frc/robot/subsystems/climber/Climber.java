/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.RobotConfigurator;
import frc.robot.resources.TecbotMotorList;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid pistonHanger;
    private final TecbotMotorList ropeController;

    private RelativeEncoder ropeEncoder;

    public Climber() {

        ropeController = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_MOTOR_PORTS,
                RobotMap.CLIMBER_INVERTED_MOTORS, RobotMap.CLIMBER_MOTOR_TYPES);
        pistonHanger = RobotConfigurator.buildDoubleSolenoid(RobotMap.CLIMBER_SOLENOID_PORTS);

        CANSparkMax leaderMotor = ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).getCANSparkMax();

        CANSparkMax slaveMotor = ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_SLAVE_PORT).getCANSparkMax();

        slaveMotor.follow( leaderMotor, true);

        // santi says break !
        leaderMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        slaveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);



        // TODO disable dragon fly in command for climber! Always!
        // TODO Dragon fly cannot be extended!!

    }

    public void setPistonExtend() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_EXTENDED_POSITION);
    }

    public DoubleSolenoid.Value getPistonState(){
        return pistonHanger.get();
    }

    public void setPistonRetract() {
        pistonHanger.set(RobotMap.CLIMBER_SOLENOID_RETRACTED_POSITION);
    }

    public void setPistonRaw(DoubleSolenoid.Value state) {
        pistonHanger.set(state);
    }

    public void setRopeControllerRaw(double speed) {

        double clampedSpeed = Math.clamp(speed, RobotMap.CLIMBER_RAW_MINIMUM_SPEED, RobotMap.CLIMBER_RAW_MAXIMUM_SPEED);
        double realSpeed = 0;

        if ( ! withinLowerRange() )
            realSpeed = Math.clamp(clampedSpeed, 0, RobotMap.CLIMBER_RAW_MAXIMUM_SPEED);
        if ( ! withinUpperRange() )
            realSpeed = Math.clamp(clampedSpeed, RobotMap.CLIMBER_RAW_MINIMUM_SPEED, 0);

        ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).set(realSpeed);
    }

    public boolean withinLowerRange(){
        return true;
    }
    public boolean withinUpperRange(){
        return true;
    }

    public TecbotMotorList getRopeControllerList() {
        return ropeController;
    }

    /**
     * Returns the master CAN Spark Max {@link RelativeEncoder}
     *
     */
    public RelativeEncoder getRopeControllerEncoder() {
        return ropeController.getSpecificMotor(RobotMap.CLIMBER_MOTOR_MASTER_PORT).getCANSparkMax().getEncoder();
    }

}