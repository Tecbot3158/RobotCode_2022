package frc.robot.commands.chassis.autonomous.splines;

import frc.robot.resources.splines.CubicSpline;
import frc.robot.resources.splines.PieceWiseSpline;
import frc.robot.resources.splines.SplineGenerator;

public class Paths {

    // TestSpline /////////////////////////////////////
    PieceWiseSpline testSpline;

    double[] testSplineXCoordinates = { 0, 2, 4 };
    double[] testSplineYCoordinates = { 0, -1, 0 };

    boolean testSplineVertical = false;

    //////////////// Five ball auto path////////////////

    PieceWiseSpline fiveBallAutoPath;

    double[] fiveBallAutoPathXCoordinates = { 1.3, 5, 7.625 };
    double[] fiveBallAutoPathYCoordinates = { 2.1, 1.95, 1.83 };


    boolean fiveBallAutoPathVertical = false;

    public Paths() {

        CubicSpline testSpline0 = SplineGenerator.GenerateSpline0(testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineYCoordinates[0], testSplineYCoordinates[1],
                testSplineYCoordinates[2]);

        CubicSpline testSpline1 = SplineGenerator.GenerateSpline1(testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineYCoordinates[0], testSplineYCoordinates[1],
                testSplineYCoordinates[2]);

        testSpline = new PieceWiseSpline(testSpline0, testSpline1, testSplineXCoordinates[0], testSplineXCoordinates[1],
                testSplineXCoordinates[2], testSplineVertical);

        CubicSpline fiveBallAutoPathSpline0 = SplineGenerator.GenerateSpline0(fiveBallAutoPathXCoordinates[0],
                fiveBallAutoPathXCoordinates[1], fiveBallAutoPathXCoordinates[2], fiveBallAutoPathYCoordinates[0],
                fiveBallAutoPathYCoordinates[1], fiveBallAutoPathYCoordinates[2]);

        CubicSpline fiveBallAutoPathSpline1 = SplineGenerator.GenerateSpline1(fiveBallAutoPathXCoordinates[0],
                fiveBallAutoPathXCoordinates[1], fiveBallAutoPathXCoordinates[2], fiveBallAutoPathYCoordinates[0],
                fiveBallAutoPathYCoordinates[1], fiveBallAutoPathYCoordinates[2]);

        fiveBallAutoPath = new PieceWiseSpline(fiveBallAutoPathSpline0, fiveBallAutoPathSpline0,
                fiveBallAutoPathXCoordinates[0], fiveBallAutoPathXCoordinates[1], fiveBallAutoPathXCoordinates[2],
                false);

    }

    public PieceWiseSpline getTestSpline() {
        return testSpline;
    }

    public PieceWiseSpline getFiveBallAutoPathSpline() {
        return fiveBallAutoPath;
    }

}
