package frc.robot.resources;

public class StepControl {

    double currentPosition = 0;
    double startPosition = 0;
    double output = 0;


    double kTarget = 0;

    double kMinimumAbsoluteOutput = 0.09;

    double incrementMultiplier = 1;

    double currentOutput = 0;


    public StepControl ( double kMinimumAbsoluteOutput, double kTarget, double kCurrentPosition,
                         double kIncrementMultiplier ){
        this.kTarget = kTarget;
        this.kMinimumAbsoluteOutput = kMinimumAbsoluteOutput;

        this.startPosition = kCurrentPosition;
        currentPosition = kCurrentPosition;

        this.incrementMultiplier = kIncrementMultiplier;

    }

    public double setCurrentPosition( double pos ){
        this.currentPosition = pos;
        return pos;
    }

    public double getOutput( double currentPosition ) {
        this.currentPosition = currentPosition;
        double proportion = ( ( kTarget - currentPosition) ) * incrementMultiplier;
        int sign = proportion >= 0 ? 1: -1;

        proportion = sign  * Math.clamp(Math.abs(proportion), kMinimumAbsoluteOutput, 1);

        currentOutput += proportion;

        return Math.clamp(currentOutput, -1, 1) ;

    }


    public double setIncrementMultiplier (double value){
        return this.incrementMultiplier = value;
    }

    public double setTarget (double target){
        return this.kTarget = target;
    }

    public double setMinAbsoluteOutput(double minOutput){
        return   this.kMinimumAbsoluteOutput = minOutput;
    }

    public double getCurrentPosition(){
        return this.currentPosition;
    }

}
