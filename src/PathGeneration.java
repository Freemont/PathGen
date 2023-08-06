import java.util.ArrayList;

/**
 * Created by Maddie Astié on 5/31/2017.
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
        ControlPolygon.add(new Waypoint(0,60,0));
        ControlPolygon.add(new Waypoint(50,70,0));
        ControlPolygon.add(new Waypoint(100,60,0));
        BetterTrajectory t1 = new BetterTrajectory(15,ControlPolygon,10,5,"kekd.txt",0.1);
        t1.GenerateSplines();
        t1.prepareTrajectory();
        t1.saveLCVRCVToCSV("lel");



        t1.toTextFile();
    }

}
