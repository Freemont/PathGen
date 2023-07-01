import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;

/**
 * Created by Maddie Astié on 5/31/2017.
 */
public class Trajectory {
    double Integral = 0;
    double totalDistance = 0;
    int count = 0;
    double lvel = 0;
    double rvel = 0;
    double lpos = 0;
    double rpos = 0;
    double runningSum = 0;
    double IntegralLeft = 0;
    double speedupDistance = 0;
    int timecounter = 0;
    double IntegralRight = 0;
    double finalSlowSpeed = 0.35;
    double currentvel = 0;
    ArrayList<Waypoint> Waypoints = new ArrayList();
    double maxvel;
    double maxaccel;
    String Filename;
    double wheelbase;
    double CurveSize = 1;
    double pixelsPerInchX = 0;
    double pixlesPerInchY = 0;
    final double SAMPLE_RATE = 1000.0; //Double for floating point math, number of integral samples per segment
    ArrayList<Segment> Segments = new ArrayList();
    double timedelta;
    public Trajectory(double Wheelbase, ArrayList Control,double maxvel, double maxaccel,String Filename,double TimeDelta) {
        Waypoints = Control;
        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
        this.Filename = Filename;
        wheelbase = Wheelbase;
        timedelta = TimeDelta;
    }
    public Trajectory(double Wheelbase, ArrayList Control,double maxvel, double maxaccel,String Filename,double TimeDelta, double ppiX,double ppiY) {
        Waypoints = Control;
        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
        this.Filename = Filename;
        wheelbase = Wheelbase;
        pixelsPerInchX = ppiX;
        pixlesPerInchY = ppiY;
        timedelta = TimeDelta;
    }
    void Generate(){
        for(int count = 0; count < Waypoints.size() -1; count ++) {

            Segments.add(new Segment(Waypoints.get(count), Waypoints.get(count + 1),CurveSize,wheelbase));

        }
    }

    void toTextFile(){
        try{
            PrintWriter writer = new PrintWriter(Filename, "UTF-8");
            for(int scount = 0; scount<Segments.size();scount++){
                if(scount == 0){

                    for(int dist = 0; dist<Segments.size();dist++) {

                        for (double time = 0; time < 1; time += 1 / SAMPLE_RATE) {
                            totalDistance += Math.sqrt(Math.pow (Segments.get(dist).calculateCenterPositionX(time+1/SAMPLE_RATE) - Segments.get(dist).calculateCenterPositionX(time),2)
                                    + Math.pow(Segments.get(dist).calculateCenterPositionY(time+1/SAMPLE_RATE) - Segments.get(dist).calculateCenterPositionY(time),2));// * 1/SAMPLE_RATE;
                        count++;
                        }
                    }

                }
                for(double t =0;t<1; t+= 1/SAMPLE_RATE) {


                    Integral += Math.sqrt(Math.pow(Segments.get(scount).calculateCenterPositionX(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateCenterPositionX(t), 2)
                            + Math.pow(Segments.get(scount).calculateCenterPositionY(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateCenterPositionY(t), 2));// * 1/SAMPLE_RATE;
                    IntegralLeft += Math.sqrt(Math.pow(Segments.get(scount).calculateLeftSplinePositionX(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateLeftSplinePositionX(t), 2)
                            + Math.pow(Segments.get(scount).calculateLeftSplinePositionY(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateLeftSplinePositionY(t), 2));// * 1/SAMPLE_RATE;
                    IntegralRight += Math.sqrt(Math.pow(Segments.get(scount).calculateRightSplinePositionX(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateRightSplinePositionX(t), 2)
                            + Math.pow(Segments.get(scount).calculateRightSplinePositionY(t + 1 / SAMPLE_RATE) - Segments.get(scount).calculateRightSplinePositionY(t), 2));// * 1/SAMPLE_RATE;


                    if (runningSum > totalDistance - speedupDistance - maxvel * timedelta) {
                        if (Integral >= currentvel * timedelta) {

                            if (Segments.get(scount).calculateCenterPositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateCenterPositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionY(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateLeftSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateLeftSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionY(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateRightSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateRightSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionY(t));
                            writer.print(",");
                            writer.print(IntegralLeft);
                            writer.print(",");
                            writer.print(IntegralRight);
                            writer.print(",");
                            writer.println(Segments.get(scount).calculateHeading(t));
                            runningSum += Integral;
                            Integral = 0;
                            currentvel -= maxaccel * timedelta;
                        }
                    } else {
                        if (Integral >= maxvel * timedelta && maxvel <= currentvel) {

                            if (Segments.get(scount).calculateCenterPositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateCenterPositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionY(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateLeftSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateLeftSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionY(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateRightSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionX(t));
                            writer.print(",");
                            if (Segments.get(scount).calculateRightSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionY(t));
                            writer.print(",");
                            writer.print(IntegralLeft);
                            writer.print(",");
                            writer.print(IntegralRight);
                            writer.print(",");
                            writer.println(Segments.get(scount).calculateHeading(t));
                            runningSum += Integral;
                            Integral = 0;
                            currentvel = maxvel;


                        } else {

                            if (Integral >= currentvel * timedelta) {

                                if (Segments.get(scount).calculateCenterPositionX(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateCenterPositionX(t));
                                writer.print(",");
                                if (Segments.get(scount).calculateCenterPositionY(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateCenterPositionY(t));
                                writer.print(",");
                                if (Segments.get(scount).calculateLeftSplinePositionX(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateLeftSplinePositionX(t));
                                writer.print(",");
                                if (Segments.get(scount).calculateLeftSplinePositionY(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateLeftSplinePositionY(t));
                                writer.print(",");
                                if (Segments.get(scount).calculateRightSplinePositionX(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateRightSplinePositionX(t));
                                writer.print(",");
                                if (Segments.get(scount).calculateRightSplinePositionY(t) != Double.NaN)
                                    writer.print(Segments.get(scount).calculateRightSplinePositionY(t));
                                writer.print(",");
                                writer.print(IntegralLeft);
                                writer.print(",");
                                writer.print(IntegralRight);
                                writer.print(",");
                                writer.println(Segments.get(scount).calculateHeading(t));
                                speedupDistance += Integral;
                                runningSum += Integral;
                                Integral = 0;
                                currentvel += maxaccel * timedelta;
                            }



                            timecounter++;

                        }

                    }
                }
                if(Integral > 0 && runningSum > totalDistance - currentvel*timedelta){

                    if(Segments.get(scount).calculateCenterPositionX(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateCenterPositionX(1));
                    writer.print(",");
                    if(Segments.get(scount).calculateCenterPositionY(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateCenterPositionY(1));
                    writer.print(",");
                    if(Segments.get(scount).calculateLeftSplinePositionX(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateLeftSplinePositionX(1));
                    writer.print(",");
                    if(Segments.get(scount).calculateLeftSplinePositionY(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateLeftSplinePositionY(1));
                    writer.print(",");
                    if(Segments.get(scount).calculateRightSplinePositionX(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateRightSplinePositionX(1));
                    writer.print(",");
                    if(Segments.get(scount).calculateRightSplinePositionY(1) != Double.NaN)
                        writer.print(Segments.get(scount).calculateRightSplinePositionY(1));
                    writer.print(",");
                    writer.print(IntegralLeft);
                    writer.print(",");
                    writer.print(IntegralRight);
                    writer.print(",");
                    writer.println(Segments.get(scount).calculateHeading(1));
                    runningSum+= Integral;
                    Integral = 0;

                }

            }
            writer.close();
        } catch (IOException e) {
            // do something
        }

    }

}
