import java.util.ArrayList;

/**
 * Created by Florent Astié on 5/31/2017.
 */
public class PathGeneration {

//Theta in radians
// x and y in whatever units you want
    //NOTE: MaxVel is the maximum velocity of the center of the robot.
    //Individual wheels may move faster than this max vel.
//Units per second don't matter for maxvel, have to be consistent with contorl points
//maxaccel dictates how fast robot can accelerate
    //file WILL be overwritten if already exists, so be cautious
    public static void main(String args[]){
        ArrayList<Waypoint> ControlPolygon = new ArrayList();
        ControlPolygon.add(new Waypoint(0,67,0));
        ControlPolygon.add(new Waypoint(45,60,-Math.PI/4));
        ControlPolygon.add(new Waypoint(60,40,-Math.PI/2));
        ControlPolygon.add(new Waypoint(60,10,-Math.PI / 2));
        Trajectory t1 = new Trajectory(15,ControlPolygon,15,2,"Path1.txt",0.1);
        t1.Generate();
        t1.toTextFile();
    }

}
