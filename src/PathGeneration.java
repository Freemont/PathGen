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
        ControlPolygon.add(new Waypoint(0,0,0));
        ControlPolygon.add(new Waypoint(10,10,-Math.PI/2.25));
        ControlPolygon.add(new Waypoint(8,3,Math.PI));
        ControlPolygon.add(new Waypoint(2,10,Math.PI / 2));
        Trajectory t1 = new Trajectory(0.5,ControlPolygon,3,10,"topkek.txt",0.05);
        t1.Generate();
        t1.toTextFile();
    }

}
