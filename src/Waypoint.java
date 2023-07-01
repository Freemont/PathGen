/**
 * Created by Maddie Astié on 5/31/2017.
 */
public class Waypoint
{
    private double x;
    private double y;
    private double theta;
    public Waypoint(double x, double y, double theta){
        this.x = x;
        this.y = y;
        this.theta = theta;
    }
    public double getX(){
        return x;
    }
    public double getY(){
        return y;
    }
    public double getTheta(){
     return theta;
    }

}
