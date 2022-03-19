package frc.robot.resources.splines;

public class PieceWiseSpline {

    CubicSpline spline0, spline1;

    Derivative derivativeSpline0, derivativeSpline1;

    double middleWaypointX;

    /**
     * Generates a new piece wise function with two cubic splines.
     * 
     * @param spline0         The first part of the piecewise function.
     * @param spline1         The second part of the piecewise function.
     * @param middleWaypointX The x value at which the first function finishes and
     *                        the second starts.
     */
    public PieceWiseSpline(CubicSpline spline0, CubicSpline spline1, double middleWaypointX) {

        this.spline0 = spline0;
        this.spline1 = spline1;
        this.middleWaypointX = middleWaypointX;

        this.derivativeSpline0 = SplineGenerator.DifferentiateSpline(spline0);
        this.derivativeSpline1 = SplineGenerator.DifferentiateSpline(spline1);

    }

    /**
     * Returns the evaluation of the function at the given point.
     * 
     * @param x The point at which the funcion will be evaluated.
     * @return Returns the evaluation of the function at the given point.
     */
    public double f(double x) {

        if (x < middleWaypointX) {
            return spline0.f(x);
        } else {
            return spline1.f(x);
        }

    }

}
