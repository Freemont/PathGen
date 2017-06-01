import java.util.ArrayList;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.io.IOException;
/**
 * Created by Florent Astié on 5/31/2017.
 */
public class Trajectory {
    double Integral = 0;
    double IntegralLeft = 0;
    double IntegralRight = 0;
    double currentvel = 0;
    ArrayList<Waypoint> Waypoints = new ArrayList();
    double maxvel;
    double maxaccel;
    String Filename;
    double wheelbase;
    double CurveSize = 1;
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
    public Trajectory(double Wheelbase, ArrayList Control,double maxvel, double maxaccel,String Filename,double TimeDelta, double CurveSize) {
        Waypoints = Control;
        this.maxvel = maxvel;
        this.maxaccel = maxaccel;
        this.Filename = Filename;
        wheelbase = Wheelbase;
        this.CurveSize = CurveSize;
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
                for(double t =0;t<1; t+= 1/SAMPLE_RATE){
                   Integral += Math.sqrt(Math.pow(Segments.get(scount).calculateCenterPositionX(t+1/SAMPLE_RATE) - Segments.get(scount).calculateCenterPositionX(t),2)
                           + Math.pow(Segments.get(scount).calculateCenterPositionY(t+1/SAMPLE_RATE) - Segments.get(scount).calculateCenterPositionY(t),2));// * 1/SAMPLE_RATE;
                    IntegralLeft += Math.sqrt(Math.pow(Segments.get(scount).calculateLeftSplinePositionX(t+1/SAMPLE_RATE) - Segments.get(scount).calculateLeftSplinePositionX(t),2)
                            + Math.pow(Segments.get(scount).calculateLeftSplinePositionY(t+1/SAMPLE_RATE) - Segments.get(scount).calculateLeftSplinePositionY(t),2));// * 1/SAMPLE_RATE;
                    IntegralRight += Math.sqrt(Math.pow(Segments.get(scount).calculateRightSplinePositionX(t+1/SAMPLE_RATE) - Segments.get(scount).calculateRightSplinePositionX(t),2)
                            + Math.pow(Segments.get(scount).calculateRightSplinePositionY(t+1/SAMPLE_RATE) - Segments.get(scount).calculateRightSplinePositionY(t),2));// * 1/SAMPLE_RATE;



                    if(Integral >= maxvel * timedelta && maxvel <= currentvel){
                        if(Segments.get(scount).calculateCenterPositionX(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateCenterPositionX(t));
                        writer.print(",");
                        if(Segments.get(scount).calculateCenterPositionY(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateCenterPositionY(t));
                        writer.print(",");
                        if(Segments.get(scount).calculateLeftSplinePositionX(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateLeftSplinePositionX(t));
                        writer.print(",");
                        if(Segments.get(scount).calculateLeftSplinePositionY(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateLeftSplinePositionY(t));
                        writer.print(",");
                        if(Segments.get(scount).calculateRightSplinePositionX(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateRightSplinePositionX(t));
                        writer.print(",");
                        if(Segments.get(scount).calculateRightSplinePositionY(t) != Double.NaN)
                            writer.print(Segments.get(scount).calculateRightSplinePositionY(t));
                        writer.print(",");
                        writer.print(IntegralLeft);
                        writer.print(",");
                       writer.println(IntegralRight);
                        Integral = 0;



                    }
                    else{
                        if(currentvel == 0){
                            currentvel += maxvel * (maxaccel/maxvel);
                        }
                        if(Integral >= currentvel * timedelta){
                            if(Segments.get(scount).calculateCenterPositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionX(t));
                            writer.print(",");
                            if(Segments.get(scount).calculateCenterPositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateCenterPositionY(t));
                            writer.print(",");
                            if(Segments.get(scount).calculateLeftSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionX(t));
                            writer.print(",");
                            if(Segments.get(scount).calculateLeftSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateLeftSplinePositionY(t));
                            writer.print(",");
                            if(Segments.get(scount).calculateRightSplinePositionX(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionX(t));
                            writer.print(",");
                            if(Segments.get(scount).calculateRightSplinePositionY(t) != Double.NaN)
                                writer.print(Segments.get(scount).calculateRightSplinePositionY(t));
                            writer.print(",");
                            writer.print(IntegralLeft);
                            writer.print(",");
                            writer.println(IntegralRight);
                            Integral = 0;
                            currentvel += maxvel * (maxaccel/maxvel);
                        }


                    }

                }
            }
            writer.close();
        } catch (IOException e) {
            // do something
        }

    }

}
