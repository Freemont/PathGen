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
    double returnLength() {
        double SAMPLE_RATE= 1000;
        double sum = 0;
        for (double t = 0; t < 1; t += 1 / SAMPLE_RATE) {


            sum += Math.sqrt(Math.pow(this.calculateCenterPositionX(t + 1 / SAMPLE_RATE) - this.calculateCenterPositionX(t), 2)
                    + Math.pow(this.calculateCenterPositionY(t + 1 / SAMPLE_RATE) - this.calculateCenterPositionY(t), 2));// * 1/SAMPLE_RATE;

        }
    return sum;
    }
    //Segments are calculated with a parameter of t from 0 to 1. xC is the x coefficients and yC is the y coefficients (of the 3rd degree spline)
    double calculateRightSplinePositionX(double time) {

        return evaluateCubic(CoeffX, time) + size / 2.0 * evaluateQuadratic(dy, time) / Math.sqrt(Math.pow(evaluateQuadratic(dx, time), 2) + Math.pow(evaluateQuadratic(dy, time), 2));
    }

    double calculateRightSplinePositionY(double time) {
        return evaluateCubic(CoeffY, time) - size / 2.0 * evaluateQuadratic(dx, time) / Math.sqrt(Math.pow(evaluateQuadratic(dx, time), 2) + Math.pow(evaluateQuadratic(dy, time), 2));
    }

    double calculateLeftSplinePositionX(double time) {
        return evaluateCubic(CoeffX, time) - size / 2.0 * evaluateQuadratic(dy, time) / Math.sqrt(Math.pow(evaluateQuadratic(dx, time), 2) + Math.pow(evaluateQuadratic(dy, time), 2));
    }

    double calculateLeftSplinePositionY(double time) {
        return evaluateCubic(CoeffY, time) + size / 2.0 * evaluateQuadratic(dx, time) / Math.sqrt(Math.pow(evaluateQuadratic(dx, time), 2) + Math.pow(evaluateQuadratic(dy, time), 2));

    }
    double calculateLeftDistance(double time, double time2) {
        double ldist = 0;
        for (double t = time; t <=time2; t += 1 / BetterTrajectory.SAMPLE_RATE){
        ldist += Math.sqrt(Math.pow(this.calculateLeftSplinePositionX(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateLeftSplinePositionX(t), 2)
                + Math.pow(this.calculateLeftSplinePositionY(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateLeftSplinePositionY(t), 2));// * 1/SAMPLE_RATE
    }
    return ldist;
    }
    double calculateRightDistance(double time, double time2) {
        double rdist = 0;
        for (double t = time; t<=time2; t += 1 / BetterTrajectory.SAMPLE_RATE);
        {
            rdist += Math.sqrt(Math.pow(this.calculateRightSplinePositionX(t + 1 / BetterTrajectory.SAMPLE_RATE) -this.calculateRightSplinePositionX(t), 2)
            +Math.pow(this.calculateRightSplinePositionY(t + 1 / BetterTrajectory.SAMPLE_RATE) - this.calculateRightSplinePositionY(t), 2));

        }
    return rdist;
    }

    double calculateLeftDistanceTime(double time){
        double dist = BetterTrajectory.maxvel * BetterTrajectory.timedelta;
return 0;
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
