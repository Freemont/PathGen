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
    int determineOutside(double t, int currentCount){
        double ldist = 0;
        double rdist = 0;
        Segment current = Segments.get(currentCount);
        if(t + 1/SAMPLE_RATE >=1){
            if(currentCount == Segments.size()-1 ){
                ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(1) - current.calculateLeftSplinePositionX(t), 2)
                        + Math.pow(current.calculateLeftSplinePositionY(1) - current.calculateLeftSplinePositionY(t), 2));
                rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(1) - current.calculateRightSplinePositionX(t), 2)
                        + Math.pow(current.calculateRightSplinePositionY(1) - current.calculateRightSplinePositionY(t), 2));
            }
            else{
                ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(1) - current.calculateLeftSplinePositionX(t), 2)
                        + Math.pow(current.calculateLeftSplinePositionY(1) - current.calculateLeftSplinePositionY(t), 2));
                rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(1) - current.calculateRightSplinePositionX(t), 2)
                        + Math.pow(current.calculateRightSplinePositionY(1) - current.calculateRightSplinePositionY(t), 2));

                Segment next = Segments.get(currentCount+1);
                ldist += Math.sqrt(Math.pow(next.calculateLeftSplinePositionX(1-t) - next.calculateLeftSplinePositionX(0), 2)
                        + Math.pow(next.calculateLeftSplinePositionY(1-t) - next.calculateLeftSplinePositionY(0), 2));
                rdist += Math.sqrt(Math.pow(next.calculateRightSplinePositionX(1-t) - next.calculateRightSplinePositionX(0), 2)
                        + Math.pow(next.calculateRightSplinePositionY(1-t) - next.calculateRightSplinePositionY(0), 2));
            }

        }
        else{
            ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionX(t), 2)
                    + Math.pow(current.calculateLeftSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionY(t), 2));
            rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionX(t), 2)
                    + Math.pow(current.calculateRightSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionY(t), 2));
        }

        if (rdist > ldist){
            return 1;
        }
        return 0;

    }

    //RIGHT WHEEL OUTSIDE = 1, LEFT = 0;
    void prepareTrajectory() {
        double RCD = 0; //right cumulative distance
        double LCD = 0; //left cumulative distance
        double RCV = 0; //right cumulative velocity
        double LCV = 0; //left cumulative velocity
        double ldist;
        double rdist;
        double lvel;
        double rvel;
        //double laccel = 0; IF NEEDED
        //double raccel = 0; IF NEEDED
        // OPTIONAL double heading = 0;
        double time = 0;
        double t=0;
        double t_temp;
        int outside;
        double t_temp2;
        // fill in cheesy poof serialization method
        traj.add(new PathPoint(0, 0, 0, 0, 0, 0, 0, 0));

        //forwards
        for (int seg = 0; seg < Segments.size(); seg++) {
            Segment current = Segments.get(seg);//run through the spline segments 1 by 1
            time += timedelta; //increment by time increment for PID controller
            outside = determineOutside(t, seg);
            rdist = 0;
            ldist = 0;
            if(outside == 1){ //RIGHT WHEEL OUTSIDE

                if(RCV < maxvel){
                    if(RCV + maxaccel > maxvel){
                        RCV = maxvel;
                    }
                    else{
                        RCV += maxaccel;
                    }
                }
                rdist = RCV* timedelta + 0.5 * maxaccel* timedelta *timedelta;
                t_temp =  current.reverseEngineerT(rdist, t, outside);
                t_temp2 = 0;
                if(t_temp < 0 && seg < Segments.size()-1) {
                    t_temp2 = Segments.get(seg + 1).reverseEngineerT(rdist - current.calculateRightDistance(t, 1), 0, outside);
                    ldist += current.calculateLeftDistance(t, 1) + Segments.get(seg + 1).calculateLeftDistance(0, t_temp2);
                }else{
                    ldist += current.calculateLeftDistance(t,t_temp);
                }
                t += t_temp + t_temp2;
                RCD += rdist;
                LCD += ldist;
                LCV = ldist*2/timedelta - LCV;
                traj.add(new PathPoint(LCD, RCD, LCV, RCV, 0, 0, 0, time));
            }

            else{//LEFT WHEEL OUTSIDE

            }
        }
        //backwards


    }
    void positionToCSV(int wheelchoice, String filename){}
    void velocityToCSV(int wheelchoice, String filename){}
    public void toTextFile() {
        for (int x = 0; x < traj.size(); x++) {
            traj.get(x).printLine();
        }

    }
}
