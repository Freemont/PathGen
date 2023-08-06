/**
 * Created by Maddie Astié on 5/31/2017.
 */
public class Segment {
    Waypoint a, b;
    double csize = 1.0; //set the derivative to be greater or smaller yields larger / smaller curve radii
    int t = 0;
    int tnot = 1;//dont touch this unless you know what you're doing

    double CoeffY[][];
    double CoeffX[][];
    double dx[][] = new double[4][1];
    double dy[][] = new double[4][1];
    double size;
    double pieceCount = 0;

    public Segment(Waypoint start, Waypoint finish, double wheelbase) {
        a = start;
        b = finish;
        size = wheelbase;
        calculateCenterSpline();
    }

    public Segment(Waypoint start, Waypoint finish, double CurveSize, double Wheelbase) {
        a = start;
        b = finish;
        csize = CurveSize;
        size = Wheelbase;
        calculateCenterSpline();
    }

    void calculateCenterSpline() {
        double XcontrolMatrix[][] = {{Math.pow(t, 3), Math.pow(t, 2), t, 1}, {Math.pow(tnot, 3), Math.pow(tnot, 2), tnot, 1}, {3 * Math.pow(t, 2), 2 * t, 1, 0}, {3 * Math.pow(tnot, 2), 2 * tnot, 1, 0}};
        double XsolutionMatrix[][] = {{a.getX()}, {b.getX()}, {angleToDx(a)}, {angleToDx(b)}};
        double Xinvert[][] = MatrixMath.invert(XcontrolMatrix);
        CoeffX = MatrixMath.multiplyByMatrix(Xinvert, XsolutionMatrix);
        double YcontrolMatrix[][] = {{Math.pow(t, 3), Math.pow(t, 2), t, 1}, {Math.pow(tnot, 3), Math.pow(tnot, 2), tnot, 1}, {3 * Math.pow(t, 2), 2 * t, 1, 0}, {3 * Math.pow(tnot, 2), 2 * tnot, 1, 0}};
        double YsolutionMatrix[][] = {{a.getY()}, {b.getY()}, {angleToDy(a)}, {angleToDy(b)}};
        double Yinvert[][] = MatrixMath.invert(YcontrolMatrix);
        CoeffY = MatrixMath.multiplyByMatrix(Yinvert, YsolutionMatrix);
        differentiate(CoeffX, dx);
        differentiate(CoeffY, dy);
    }


    //return the associated T value with a given distance, return -1 if runs into the end of a segment

    double reverseEngineerT(double distance, double t1, double outside){
        double precision = 0.003; //only accept t if within this range of the true distance
        double distance_guess = 0;

        while(true) {
            double delta = (1 - t1) / 2; //The amount to change t1 by each time (also the initial interval value between t1 and the current guess)
            double guess = t1 + delta; //Calculating the new guess value for t

            //Determining the distance traveled by the wheel (whichever one is outside)  from t1 to the current guess value
            if(outside ==0){
                 distance_guess = calculateLeftDistance(t1, guess);
            }
            else{
                 distance_guess = calculateRightDistance(t1, guess);
            }

            //If the distance guess is within the acceptable range, return the guess as the approximate t value
            if (distance_guess > distance - precision && distance_guess < distance + precision) {
                return guess;
            } else if(delta < precision/2) {
                //If delta becomes smaller than half of the precision value, then return saying it couldn't find an acceptable t value
               return -1;
            } else {
                //Refine the guess value for t based on whether the distance is greater or smaller than distance_guess
                delta /= 2;
                if (distance > distance_guess) {
                    guess += delta;
                } else {
                    guess -= delta;
                }
            }
        }
    }

    //Segments are calculated with a parameter of t from 0 to 1. xC is the x coefficients and yC is the y coefficients (of the 3rd degree spline)
    double calculateRightSplinePositionX(double t) {

        return evaluateCubic(CoeffX, t) + size / 2.0 * evaluateQuadratic(dy, t) / Math.sqrt(Math.pow(evaluateQuadratic(dx, t), 2) + Math.pow(evaluateQuadratic(dy, t), 2));
    }

    double calculateRightSplinePositionY(double t) {
        return evaluateCubic(CoeffY, t) - size / 2.0 * evaluateQuadratic(dx, t) / Math.sqrt(Math.pow(evaluateQuadratic(dx, t), 2) + Math.pow(evaluateQuadratic(dy, t), 2));
    }

    double calculateLeftSplinePositionX(double t) {
        return evaluateCubic(CoeffX, t) - size / 2.0 * evaluateQuadratic(dy, t) / Math.sqrt(Math.pow(evaluateQuadratic(dx, t), 2) + Math.pow(evaluateQuadratic(dy, t), 2));
    }

    double calculateLeftSplinePositionY(double t) {
        return evaluateCubic(CoeffY, t) + size / 2.0 * evaluateQuadratic(dx, t) / Math.sqrt(Math.pow(evaluateQuadratic(dx, t), 2) + Math.pow(evaluateQuadratic(dy, t), 2));

    }
    double calculateLeftDistance(double t1, double t2) {
        double ldist = 0;
        for (double t = t1; t <= t2; t += (1 / BetterTrajectory.SAMPLE_RATE)){
        ldist += Math.sqrt(Math.pow(this.calculateLeftSplinePositionX(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateLeftSplinePositionX(t), 2)
                + Math.pow(this.calculateLeftSplinePositionY(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateLeftSplinePositionY(t), 2));// * 1/SAMPLE_RATE
    }
    return ldist;
    }
    double calculateRightDistance(double t1, double t2) {
        double rdist = 0;
        for (double t = t1; t<=t2; t += 1 / BetterTrajectory.SAMPLE_RATE);
        {
            rdist += Math.sqrt(Math.pow(this.calculateRightSplinePositionX(t + 1 / BetterTrajectory.SAMPLE_RATE) -this.calculateRightSplinePositionX(t), 2)
            +Math.pow(this.calculateRightSplinePositionY(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateRightSplinePositionY(t), 2));

        }
    return rdist;
    }


    double calculateCenterPositionX(double time) {
        return evaluateCubic(CoeffX, time);
    }

    double calculateCenterPositionY(double time) {
        return evaluateCubic(CoeffY, time);
    }

    double calculateHeading(double time) {

    double deltaX = calculateLeftSplinePositionX(time) - calculateRightSplinePositionX(time);
    double deltaY = calculateLeftSplinePositionY(time) - calculateRightSplinePositionY(time);
    return  Math.atan2(deltaY,deltaX) * 180 / Math.PI -90;
}
    //@param The waypoint specified MUST be either the start or finish waypoint for this segment
    double angleToDx(Waypoint waypoint){
        return csize* Math.cos(waypoint.getTheta()) * Math.sqrt((Math.pow(b.getX() - a.getX() ,2)+Math.pow(b.getY() - a.getY(),2)));
    }
    double angleToDy(Waypoint waypoint){
        return csize* Math.sin(waypoint.getTheta()) * Math.sqrt((Math.pow(b.getX() - a.getX() ,2)+Math.pow(b.getY() - a.getY(),2)));
    }
    void differentiate(double Coefficients[][],double target[][]){
        target[0][0] = Coefficients[0][0] * 3;
        target[1][0] = Coefficients[1][0] * 2;
        target[2][0] = Coefficients[2][0];
        target[3][0] = 0;

    }
    double evaluateCubic(double[][] Coeffs, double t){
        return Coeffs[0][0]*Math.pow(t,3)  + Coeffs[1][0]*Math.pow(t,2) + Coeffs[2][0]*Math.pow(t,1) + Coeffs[3][0];
    }
    double evaluateQuadratic(double[][] Coeffs, double t){
        return Coeffs[0][0]*Math.pow(t,2)  + Coeffs[1][0]*Math.pow(t,1) + Coeffs[2][0]*Math.pow(t,0);
    }
}
