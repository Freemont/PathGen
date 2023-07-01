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
    }

    void GenerateSplines() {
        for (int count = 0; count < Waypoints.size() - 1; count++) {

            Segments.add(new Segment(Waypoints.get(count), Waypoints.get(count + 1), CurveSize, wheelbase));

        }
    }

    double getTotalDistance() {
        double sum = 0;
        for (Segment seg : Segments) {
            for (double t = 0; t < 1; t += 1 / SAMPLE_RATE) {


                sum += Math.sqrt(Math.pow(seg.calculateCenterPositionX(t + 1 / SAMPLE_RATE) - seg.calculateCenterPositionX(t), 2)
                        + Math.pow(seg.calculateCenterPositionY(t + 1 / SAMPLE_RATE) - seg.calculateCenterPositionY(t), 2));// * 1/SAMPLE_RATE;

            }
        }
        return sum;
    }

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

    void calculateTarget(double y, Segment current, int currentCount) {
        double dr = 0;
        double dl = 0;
        double t = y;
        if (outside == 1) {
            if (rvel + maxaccel * timedelta > maxvel) {
                rightTarget = rvel * timedelta;
            } else {
                rightTarget = rvel * timedelta + 0.5 * maxaccel * Math.pow(timedelta, 2);
                rvel = rvel + maxaccel* timedelta;
            }
            while (dr < rightTarget) {
                dr += Math.sqrt(Math.pow(current.calculateRightSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionX(t), 2)
                        + Math.pow(current.calculateRightSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionY(t), 2));
                t += 1 / SAMPLE_RATE;
                if (t == 1) {
                    if (currentCount >= Segments.size() - 1) {
                        break;
                    } else {
                        t = 0;
                        current = Segments.get(currentCount + 1);
                    }
                }

            }


            if (t < y) {
                elapsedSinceLast = 1 + t - y;

            } else {
                elapsedSinceLast = t - y;
            }
        }
        //////////////////////////////////////////////////////////

        if (outside == 0) {
            if (lvel + maxaccel * timedelta > maxvel) {
                leftTarget = lvel * timedelta;
            } else {
                leftTarget = lvel * timedelta + 0.5 * maxaccel * Math.pow(timedelta, 2);
                lvel = lvel + maxaccel * timedelta;
            }
            while (dl < leftTarget) {
                dl += Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionX(t), 2)
                        + Math.pow(current.calculateLeftSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionY(t), 2));
                t += 1 / SAMPLE_RATE;
                if (t == 1) {
                    if (currentCount >= Segments.size() - 1) {
                        break;
                    } else {
                        t = 0;
                        current = Segments.get(currentCount + 1);
                    }
                }

            }
            if (t < y) {
                elapsedSinceLast = 1 + t - y;

            } else {
                elapsedSinceLast = t - y;
            }
        }





    }



    //RIGHT WHEEL OUTSIDE = 1, LEFT = 0;
            void prepareTrajectory() {
        double toNext;
        double elapsed = 0;
        double ldist,rdist,laccel,raccel,heading,time;
       ldist = 0;
       rdist = 0;
       double rcumulative = 0;
       double lcumulative = 0;
       lvel = 0;
       rvel = 0;
       laccel = 0;
       raccel = 0;
       heading = 0;
       time = 0;

double oldLeft, oldRight;
    traj.add(new PathPoint(0,0,0,0,0,0,0,0));
    for(int seg = 0; seg < Segments.size(); seg++) {
        Segment current = Segments.get(seg);

        while (elapsed < 1) {
            oldLeft = lcumulative;
            oldRight = rcumulative;
            determineOutside(lvel, rvel, seg, elapsed);
            calculateTarget(elapsed, current, seg);
            if(elapsed + elapsedSinceLast > 1){
                if (outside == 1) {

                    rcumulative += current.calculateRightDistance(elapsed,1) + Segments.get(seg+1).calculateRightDistance(0,elapsed + elapsedSinceLast - 1);
                    lcumulative += current.calculateLeftDistance(elapsed, 1) + Segments.get((seg+1)).calculateLeftDistance(0,-1+elapsed+elapsedSinceLast);
                        lvel = (lcumulative-oldLeft) / timedelta;

                }
                else {
                    rcumulative += current.calculateRightDistance(elapsed,1) + Segments.get(seg+1).calculateRightDistance(0,elapsed + elapsedSinceLast - 1);
                    lcumulative += current.calculateLeftDistance(elapsed, 1) + Segments.get((seg+1)).calculateLeftDistance(0,-1+elapsed+elapsedSinceLast);
                    rvel = (rcumulative - oldRight / timedelta);



                    //  x = vo * t + 1/2 a * t ^2
                }
            }
            else {
                if (outside == 1) {
                    rcumulative += current.calculateRightDistance(elapsed,elapsedSinceLast + elapsed);
                    lcumulative += current.calculateLeftDistance(elapsed, elapsedSinceLast + elapsed);
                    System.out.println("Right");
                    lvel = (lcumulative-oldLeft) / timedelta;

                } else {
                    lcumulative += current.calculateLeftDistance(elapsed,elapsedSinceLast+elapsed);
                    rcumulative += current.calculateRightDistance(elapsed, elapsedSinceLast+elapsed);
                    rvel = (rcumulative-oldRight) / timedelta;
                    System.out.println("Left");
                    //  x = vo * t + 1/2 a * t ^2
                }

            }
            elapsed += elapsedSinceLast;
            traj.add(new PathPoint(lcumulative,rcumulative,lvel,rvel,0,0,0,time));
            time += timedelta;
        }


    }
}

    void toTextFile() {
for(int x = 0; x < traj.size(); x++){
    traj.get(x).printLine();
}

    }

}
