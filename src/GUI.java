/**
 * Created by Florent Astié on 5/31/2017.
 */
import java.awt.*;
import java.awt.event.*;
import java.util.ArrayList;
import javax.swing.*;

import static java.awt.Color.DARK_GRAY;

public class GUI extends JFrame implements ActionListener,MouseListener{
JLabel field;
JTextArea filein;
double xInput,yInput;
JLabel filepath;
JButton reset;
JLabel feedback;
JLabel angInput;
JLabel totalWaypoints;
JButton export;
JButton addWaypoint;
JTextArea angleInput;
double pixelsPerInchY = 740 / 12.0 / 27;
double pixelsPerInchX = 1490 / 12.0 / 54;
ArrayList<Waypoint> waypoints = new ArrayList();
Trajectory test = new Trajectory(3*12,waypoints,3,3,"Testpath",0.05);
    public GUI(){
        super("Trajectory Planner");
        setSize(1700,740);
        setVisible(true);
        SpringLayout layout = new SpringLayout();
        setLayout(layout);
        ImageIcon icon = new ImageIcon("SteamworksField.png");
         field = new JLabel(icon);
         angInput = new JLabel("Angle in degrees: ");
         feedback = new JLabel();
         filein = new JTextArea(1,10);
         filepath = new JLabel("Target filename: ");
         totalWaypoints = new JLabel("Total Waypoints: 0");
         export = new JButton("To text file!");
         reset = new JButton("Reset");
         addWaypoint = new JButton("Add Waypoint");
         angleInput = new JTextArea(1,5);
        reset.setVisible(true);

        field.setBounds(0, 0, 1490, 740);
        add(filepath);
        add(field);
        add(angInput);
        add(reset);
        add(angleInput);
        add(totalWaypoints);
        add(feedback);
        add(addWaypoint);
        add(filein);
        add(export);
        export.addActionListener(this);
        reset.addActionListener(this);
        addWaypoint.addActionListener(this);
        field.addMouseListener(this);
        layout.putConstraint(SpringLayout.WEST, field, 5, SpringLayout.WEST, this);
        layout.putConstraint(SpringLayout.NORTH, field, 5, SpringLayout.NORTH, this);
        layout.putConstraint(SpringLayout.WEST, reset,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, reset, 100, SpringLayout.NORTH, field);
        layout.putConstraint(SpringLayout.WEST, feedback,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, feedback, 5, SpringLayout.NORTH, field);
        layout.putConstraint(SpringLayout.WEST, angleInput,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, angleInput, -20, SpringLayout.NORTH, reset);
        layout.putConstraint(SpringLayout.WEST, addWaypoint,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, addWaypoint, 5, SpringLayout.SOUTH, reset);
        layout.putConstraint(SpringLayout.WEST, angInput,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, angInput, -25, SpringLayout.NORTH, angleInput);
        layout.putConstraint(SpringLayout.WEST, export,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, export, 35, SpringLayout.NORTH, addWaypoint);
        layout.putConstraint(SpringLayout.WEST, totalWaypoints,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, totalWaypoints, 35, SpringLayout.NORTH, export);
        layout.putConstraint(SpringLayout.WEST, filepath,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, filepath, 20, SpringLayout.NORTH, totalWaypoints);
        layout.putConstraint(SpringLayout.WEST, filein,1490 +15,SpringLayout.WEST, field);
        layout.putConstraint(SpringLayout.NORTH, filein, 15, SpringLayout.NORTH, filepath);




        // add the button to the JFrame
        setDefaultCloseOperation( JFrame.EXIT_ON_CLOSE );


    }
    public void paint(Graphics g){
        super.paint(g);
        Graphics2D g2 = (Graphics2D) g;

        for(int scount = 0; scount<test.Segments.size();scount++){

            for(double t =0;t<1; t+= 1/test.SAMPLE_RATE) {

                Double xin = test.Segments.get(scount).calculateCenterPositionX(t) * pixelsPerInchX;
                Double yin = test.Segments.get(scount).calculateCenterPositionY(t)  * pixelsPerInchY;
                Double xin2 = test.Segments.get(scount).calculateCenterPositionX(t+ 1/test.SAMPLE_RATE)  * pixelsPerInchX;
                Double yin2 = test.Segments.get(scount).calculateCenterPositionY(t+ 1/test.SAMPLE_RATE)  * pixelsPerInchY;
                Double xin3 =   test.Segments.get(scount).calculateLeftSplinePositionX(t)  * pixelsPerInchX;
                Double yin3 = test.Segments.get(scount).calculateLeftSplinePositionY(t) * pixelsPerInchY;
                Double xin4 = test.Segments.get(scount).calculateLeftSplinePositionX(t+ 1/test.SAMPLE_RATE)  * pixelsPerInchX;
                Double yin4= test.Segments.get(scount).calculateLeftSplinePositionY(t+ 1/test.SAMPLE_RATE) * pixelsPerInchY;
                Double xin5 =test.Segments.get(scount).calculateRightSplinePositionX(t)  * pixelsPerInchX;
                Double yin5 = test.Segments.get(scount).calculateRightSplinePositionY(t)  * pixelsPerInchY;
                Double xin6 =test.Segments.get(scount).calculateRightSplinePositionX(t+ 1/test.SAMPLE_RATE)  * pixelsPerInchX;
                Double yin6 = test.Segments.get(scount).calculateRightSplinePositionY(t+ 1/test.SAMPLE_RATE)  * pixelsPerInchY;

                Double t1 = test.Waypoints.get(scount).getX() * pixelsPerInchX;
                Double t2 =  test.Waypoints.get(scount).getY() * pixelsPerInchY;
                Double t3 =  test.Waypoints.get(scount).getX() * pixelsPerInchX;
                Double t4 =  test.Waypoints.get(scount).getY() * pixelsPerInchY;

                //g2.drawLine(t1.intValue(),t2.intValue(),t3.intValue(),t4.intValue());
                g2.setColor(Color.GREEN);
                g2.setStroke(new BasicStroke(2));
                g2.drawLine(xin.intValue() + 10, yin.intValue() + 45, xin2.intValue() + 10, yin2.intValue() + 45);
                g2.setColor(Color.MAGENTA);
                g2.drawLine(xin3.intValue() + 10, yin3.intValue() + 45, xin4.intValue() + 10, yin4.intValue() + 45);
                g2.drawLine(xin5.intValue() + 10, yin5.intValue() + 45, xin6.intValue() + 10, yin6.intValue() + 45);

            }
        }


    }

    public void actionPerformed( ActionEvent evt)
    {
        if(evt.getSource()==reset){
        test.Waypoints.clear();
        test.Segments.clear();
        totalWaypoints.setText("Total waypoints: "+ test.Waypoints.size());
        repaint();
    }
        if(evt.getSource()==export){
            test.Filename = filein.getText() + ".txt";

           test.toTextFile();
            filein.setText("");
           feedback.setText("Wrote path to text file.");
           test.Waypoints.clear();
           test.Segments.clear();
           repaint();
        }

        if(evt.getSource() == addWaypoint){
            test.Waypoints.add(new Waypoint((double)(xInput),(double)(yInput),-Math.toRadians(Double.parseDouble(angleInput.getText()))));
            feedback.setText("<html>Added Waypoint!<br>Click next waypoint location.</html>");
            totalWaypoints.setText("Total waypoints: "+ test.Waypoints.size());
            test.Segments.clear();
            test.Generate();
            if(test.Segments.size() >= 1){

                repaint();

                //feedback.setText(String.valueOf(test.Segments.size()));
            }
        }
    }
    public static void main(String[] args){
        GUI  gui = new GUI();
            gui.setVisible(true);

    }

    @Override
    public void mouseClicked(MouseEvent e) {

        feedback.setText(e.getX()+" , " + e.getY());
        xInput = e.getX() / pixelsPerInchX;
        yInput = e.getY() / pixelsPerInchY;


    }

    @Override
    public void mousePressed(MouseEvent e) {

    }

    @Override
    public void mouseReleased(MouseEvent e) {

    }

    @Override
    public void mouseEntered(MouseEvent e) {

    }

    @Override
    public void mouseExited(MouseEvent e) {

    }
}
