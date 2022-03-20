package frc.robot.commands.chassis.autonomous.splines;

import frc.robot.resources.splines.CubicSpline;
import frc.robot.resources.splines.PieceWiseSpline;
import frc.robot.resources.splines.SplineGenerator;

public class Paths {

    // TestSpline /////////////////////////////////////
    PieceWiseSpline testSpline;

    double[] testSplineXCoordinates = { 0, 2, 4 };
    double[] testSplineYCoordinates = { 0, 1, 0 };

    boolean testSplineVertical = false;

    public Paths() {

        CubicSpline testSpline0 = SplineGenerator.GenerateSpline0(testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineYCoordinates[0], testSplineYCoordinates[1],
                testSplineYCoordinates[2]);

        CubicSpline testSpline1 = SplineGenerator.GenerateSpline1(testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineYCoordinates[0], testSplineYCoordinates[1],
                testSplineYCoordinates[2]);

        testSpline = new PieceWiseSpline(testSpline0, testSpline1, testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineVertical);

    }

    public PieceWiseSpline getTestSpline() {
        return testSpline;
    }

}
