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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.resources.Math;
import frc.robot.resources.*;

public class Climber extends SubsystemBase {

    private final DoubleSolenoid pistonHanger;
    private final TecbotMotorList ropeController;

   public Climber() {

       ropeController = RobotConfigurator.buildMotorList(RobotMap.CLIMBER_MOTOR_PORTS,  RobotMap.CLIMBER_INVERTED_MOTORS, RobotMap.CLIMBER_MOTOR_TYPES );
       pistonHanger = RobotConfigurator.buildDoubleSolenoid(RobotMap.CLIMBER_SOLENOID_PORTS);

   }

   public void setPistonExtend(){
       pistonHanger.set(RobotMap.CLIMBER_SOLENOID_EXTENDED_POSITION );
   }

   public void setPistonRetract(){
       pistonHanger.set(RobotMap.CLIMBER_SOLENOID_RETRACTED_POSITION );
   }

   public void setPistonRaw(DoubleSolenoid.Value state){
       pistonHanger.set(state);
   }

   public void setRopeControllerRaw(double speed){
       ropeController.setAll(speed);
   }

   public TecbotMotorList getRopeControllerList(){
       return ropeController;
   }

   /**
   Returns a CAN Spark Max {@link RelativeEncoder}
    @param index index of the MotorList to retrieve the encoder from.
    */
   public RelativeEncoder getRopeControllerEncoder(int index){
       return ropeController.getMotors().get(index).getCANSparkMax().getEncoder();
   }



}