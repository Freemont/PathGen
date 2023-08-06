/**
 * Created by Maddie Astié on 6/17/2017.
 */
public class PathPoint {
    double ldist;
    double rdist;
    double lvel;
    double rvel;
    double laccel;
    double raccel;
    double time;
    double heading;
    public PathPoint(double leftdist, double rightdist, double leftvel, double rightvel,double leftaccel, double rightaccel, int heading, double time){
        ldist = leftdist;
        rdist = rightdist;
        lvel = leftvel;
        rvel = rightvel;
        laccel = leftaccel;
        raccel = rightaccel;
        this.heading = heading;
        this.time = time;
    }
    void printLine(){

        System.out.println(ldist + "," + rdist + "," + lvel +"," + rvel + "," + laccel+ "," + raccel + "," + heading + "," + time );

    }



}
