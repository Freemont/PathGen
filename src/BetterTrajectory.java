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
        Segment current = Segments.get(currentCount); //see which segment we're on

        //Check if the current time + one SAMPLE_RATE (in the future) is greater than the time it takes to complete the segment (1)
        if(t + 1/SAMPLE_RATE >=1){
            if(currentCount == Segments.size()-1 ){
                //If this is the last segment, calculate the distances from t to the end (1)
                ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(1) - current.calculateLeftSplinePositionX(t), 2)
                        + Math.pow(current.calculateLeftSplinePositionY(1) - current.calculateLeftSplinePositionY(t), 2));
                rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(1) - current.calculateRightSplinePositionX(t), 2)
                        + Math.pow(current.calculateRightSplinePositionY(1) - current.calculateRightSplinePositionY(t), 2));
            }
            else{
                //If this is not the last segment, calculate the distances from t to the end (1) and from the beginning (0) to t (this is due to us going 1 sample rate into the future and needing to correctly see the total distance from the start to the end of the interval)
                ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(1) - current.calculateLeftSplinePositionX(t), 2)
                        + Math.pow(current.calculateLeftSplinePositionY(1) - current.calculateLeftSplinePositionY(t), 2));
                rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(1) - current.calculateRightSplinePositionX(t), 2)
                        + Math.pow(current.calculateRightSplinePositionY(1) - current.calculateRightSplinePositionY(t), 2));

                //This is just calculating the distances in the next segment in our time interval from 0 to 1-t (so we get the full distance)
                Segment next = Segments.get(currentCount+1);
                ldist += Math.sqrt(Math.pow(next.calculateLeftSplinePositionX(1-t) - next.calculateLeftSplinePositionX(0), 2)
                        + Math.pow(next.calculateLeftSplinePositionY(1-t) - next.calculateLeftSplinePositionY(0), 2));
                rdist += Math.sqrt(Math.pow(next.calculateRightSplinePositionX(1-t) - next.calculateRightSplinePositionX(0), 2)
                        + Math.pow(next.calculateRightSplinePositionY(1-t) - next.calculateRightSplinePositionY(0), 2));
            }

        }
        else{
            //If we are not going to be at the end of a segment going one SAMPLE_RATE into the future, just calculate the distance from t to t+1/SAMPLE_RATE (this is our interval)
            ldist = Math.sqrt(Math.pow(current.calculateLeftSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionX(t), 2)
                    + Math.pow(current.calculateLeftSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateLeftSplinePositionY(t), 2));
            rdist = Math.sqrt(Math.pow(current.calculateRightSplinePositionX(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionX(t), 2)
                    + Math.pow(current.calculateRightSplinePositionY(t + 1 / SAMPLE_RATE) - current.calculateRightSplinePositionY(t), 2));
        }

        //Comparing to see which wheel is the outside wheel and appropriately setting the value to either 0 or 1 (even if they wheels are going in a straight line, setting one or the other as the outside wheel will be OK)
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
        //double laccel = 0; IF NEEDED
        //double raccel = 0; IF NEEDED
        // OPTIONAL double heading = 0;
        double time = 0;
        double t=0;
        double t_temp;
        int outside;
        double t_temp2;
        // fill in cheesy poof serialization method

        //adding the initial / start path point with zero for all values
        traj.add(new PathPoint(0, 0, 0, 0, 0, 0, 0, 0));

        //Calculating the trajectory points for each segment going forward
        for (int seg = 0; seg < Segments.size(); seg++) {
            Segment current = Segments.get(seg);//run through the spline segments 1 by 1 (this is getting the current segment we will be "working" on)
            time += timedelta; //Incrementing by a time increment for PID controller
            outside = determineOutside(t, seg); //Determining which wheel is the outside wheel for this time interval inside the segment
            rdist = 0;
            ldist = 0;
            if(outside == 1){ //RIGHT WHEEL OUTSIDE

                //Calculate the right wheel velocity if velocity is not already maxed out
                if(RCV < maxvel){
                    if(RCV + maxaccel > maxvel){
                        RCV = maxvel;
                    }
                    else{
                        RCV += maxaccel;
                    }
                }

                //Calculating the right wheel distance using a Kinematics equation: d = v*t + 0.5*a*t^2
                rdist = RCV* timedelta + 0.5 * maxaccel* timedelta *timedelta;

                //Brute force the t value associated with the distance calculated by repetitively guessing.
                t_temp =  current.reverseEngineerT(rdist, t, outside);
                t_temp2 = 0;

                //Calculating the left wheel distance as it is the "inside" wheel
                //If t_temp is < 0 (essentially -1), then we are going to go beyond the current segment and need to calculate the time in the next segment as well to fully complete the distance traveled (rdist)
                if(t_temp < 0 && seg < Segments.size()-1) {
                    t_temp2 = Segments.get(seg + 1).reverseEngineerT(rdist - current.calculateRightDistance(t, 1), 0, outside);
                    ldist += current.calculateLeftDistance(t, 1) + Segments.get(seg + 1).calculateLeftDistance(0, t_temp2); //This is where t_temp2 it is the time in the next segment for the wheel and is used to find the distance for the wheel in the second segment
                }else{
                    //If t_temp is > 0, then we are going to stay in the current segment and just calculate the distance traveled in the current segment
                    ldist += current.calculateLeftDistance(t,t_temp);
                }

                //Updating t for the next for loop iteration
                t += t_temp + t_temp2; //t_temp2 is to find the time in the next segment corresponding to the remaining distance / distance traveled in the next segment
                if(t > 1) t-=1; //remove 1 if we are into the next segment
                //Updating the cumulative distances (RCD and LCD) as they are accumulators
                RCD += rdist;
                LCD += ldist;

                //Calculating the left wheel velocity using a Kinematics equation: d= (vi + vf)/2 * t and updating the cumulative velocity (LCV)
                LCV = ldist*2/timedelta - LCV;

                //Adding the data for that trajectory point to the list
                traj.add(new PathPoint(LCD, RCD, LCV, RCV, 0, 0, 0, time));
            }

            else{//LEFT WHEEL OUTSIDE

                //Calculate the left wheel velocity if velocity is not already maxed out
                if(LCV < maxvel){
                    if(LCV + maxaccel > maxvel){
                        LCV = maxvel;
                    }
                    else{
                        LCV += maxaccel;
                    }
                }

                //Calculating the left wheel distance using a Kinematics equation: d = v*t + 0.5*a*t^2
                ldist = LCV* timedelta + 0.5 * maxaccel* timedelta *timedelta;

                //Brute force the t value associated with the distance calculated by repetitively guessing.
                t_temp =  current.reverseEngineerT(ldist, t, outside);
                t_temp2 = 0;

                //Calculating the right wheel distance as it is the "inside" wheel
                //If t_temp is < 0 (essentially -1), then we are going to go beyond the current segment and need to calculate the time in the next segment as well to fully complete the distance traveled (ldist)
                if(t_temp < 0 && seg < Segments.size()-1) {
                    t_temp2 = Segments.get(seg + 1).reverseEngineerT(ldist - current.calculateLeftDistance(t, 1), 0, outside);
                    rdist += current.calculateRightDistance(t, 1) + Segments.get(seg + 1).calculateRightDistance(0, t_temp2); //This is where t_temp2 it is the time in the next segment for the wheel and is used to find the distance for the wheel in the second segment
                }else{
                    //If t_temp is > 0, then we are going to stay in the current segment and just calculate the distance traveled in the current segment
                    rdist += current.calculateRightDistance(t,t_temp);
                }

                //Updating t for the next for loop iteration
                t += t_temp + t_temp2; //t_temp2 is to find the time in the next segment corresponding to the remaining distance / distance traveled in the next segment
                if(t > 1) t-=1; //remove 1 if we are into the next segment
                //Updating the cumulative distances (LCD and RCD) as they are accumulators
                LCD += ldist;
                RCD += rdist;


                //Calculating the right wheel velocity using a Kinematics equation: d= (vi + vf)/2 * t and updating the cumulative velocity (RCV)
                RCV = rdist*2/timedelta - RCV;

                //Adding the data for that trajectory point to the list
                traj.add(new PathPoint(LCD, RCD, LCV, RCV, 0, 0, 0, time));

            }
        }
        //backwards
    }
    public void saveLCVRCVToCSV(String filename){
        try {
            FileWriter fileWriter = new FileWriter(filename);
            PrintWriter printWriter = new PrintWriter(fileWriter);

            printWriter.println("LCV,RCV"); //CSV Header column names

            for(PathPoint point : traj) { //Writing the LCV and RCV Values as CSV Rows
                printWriter.println(point.lvel + "," + point.rvel); //It is lvel and rvel because there is a velocity at that specific point, not a cumulative velocity
            }

            printWriter.close();
            System.out.println("LCV and RCV data saved to " + filename + " successfully.");
        } catch (IOException e) {
            e.printStackTrace();
            System.err.println("Error while saving LCV and RCV data to " + filename + ".");
        }
    }

    public void toTextFile() {
        for (int x = 0; x < traj.size(); x++) {
            traj.get(x).printLine();
        }

    }
}
