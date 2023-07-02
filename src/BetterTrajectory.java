import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;

/**
 * Created by Maddie Astié on 5/31/2017.
 */
public class BetterTrajectory {
    ArrayList<PathPoint> traj = new ArrayList();
    double Integral = 0;
    double totalDistance = 0;
    double lvel = 0;
    int outside = 3;
    double leftTarget;
    double rightTarget;
    double elapsedSinceLast = 0;
    double rvel = 0;
    double lpos = 0;
    double rpos = 0;
    int count = 0;
    double runningSum = 0;
    int numPoints = 10000;
    double IntegralLeft = 0;
    double speedupDistance = 0;
    int timecounter = 0;
    double IntegralRight = 0;
    double finalSlowSpeed = 0.35;
    double currentvel = 0;
    ArrayList<Waypoint> Waypoints = new ArrayList();
    static double maxvel;
    double maxaccel;
    String Filename;
    double wheelbase;
    double CurveSize = 1;
    double pixelsPerInchX = 0;
    double pixlesPerInchY = 0;
    public static final double SAMPLE_RATE = 1000.0; //Double for floating point math, number of integral samples per segment
    ArrayList<Segment> Segments = new ArrayList();
    static double timedelta;

    public BetterTrajectory(double Wheelbase, ArrayList Control, double maxvel, double maxaccel, String Filename, double TimeDelta) {
        Waypoints = Control;
        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
        this.Filename = Filename;
        wheelbase = Wheelbase;
        timedelta = TimeDelta;
        this.GenerateSplines();
    }

    public BetterTrajectory(double Wheelbase, ArrayList Control, double maxvel, double maxaccel, String Filename, double TimeDelta, double ppiX, double ppiY) {
        Waypoints = Control;
        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
        this.Filename = Filename;
        wheelbase = Wheelbase;
        pixelsPerInchX = ppiX;
        pixlesPerInchY = ppiY;
        timedelta = TimeDelta;
        this.GenerateSplines();
    }

    void GenerateSplines() {
        for (int count = 0; count < Waypoints.size() - 1; count++) {

            Segments.add(new Segment(Waypoints.get(count), Waypoints.get(count + 1), CurveSize, wheelbase));

        }
    }


    //determine which wheel is on the outside for a given interval of time
    //right = 1
    //left = 0
    void determineOutside(double lvel, double rvel, int currentCount, double y) {
        double ldist = 0;
        double t = y;
        double rdist = 0;
        double maxdist = maxvel * timedelta;

        Segment current = Segments.get(currentCount);
        while (rdist < maxdist && ldist < maxdist) {
            ldist += Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionX(t), 2)
                    + Math.pow(current.calculateLeftSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionY(t), 2));// * 1/SAMPLE_RATE
            rdist += Math.sqrt(Math.pow(current.calculateRightSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionX(t), 2)
                    + Math.pow(current.calculateRightSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionY(t), 2));
            t += 1 / SAMPLE_RATE;
            if (t >= 1) {
                if (currentCount >= Segments.size() - 1) {
                    break;
                } else {
                    t = 0;
                    current = Segments.get(currentCount + 1);
                }
            }
        }
        if (rdist > maxdist) {
            outside = 1;
        } else {
            outside = 0;
        }
    }

    //RIGHT WHEEL OUTSIDE = 1, LEFT = 0;
    void prepareTrajectory() {
        double toNext;
        double elapsed = 0;
        double ldist = 0;
        double dist = 0;
        double rcumulative = 0;
        double lcumulative = 0;
        double lvel = 0;
        double rvel = 0;
        double laccel = 0;
        double raccel = 0;
        // OPTIONAL double heading = 0;
        double time = 0;
        // fill in cheesy poof seraliization method
        traj.add(new PathPoint(0, 0, 0, 0, 0, 0, 0, 0));
        for (int seg = 0; seg < Segments.size(); seg++) {
            Segment current = Segments.get(seg);
            //run through the spline segments 1 by 1

        }


    }

    public void toTextFile() {
        for (int x = 0; x < traj.size(); x++) {
            traj.get(x).printLine();
        }

    }
}
