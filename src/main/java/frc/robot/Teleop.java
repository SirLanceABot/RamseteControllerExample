package frc.robot;

import java.lang.invoke.MethodHandles;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.component.DrivetrainFalconFX;
// import frc.component.DrivetrainSparkMax;

public class Teleop 
{
    private static int teleopCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    private static final DrivetrainFalconFX drivetrain = Robot.DRIVETRAIN;
    private static final XboxController driverController = Robot.DRIVER_CONTROLLER;
    
    
    // *** STATIC INITIALIZATION BLOCK ****************************************
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println(Constants.CLASS_LOADING + fullClassName);
    }


    public Teleop()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(teleopCounter >= Constants.TELEOP_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            teleopCounter++;
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }
    
    public void init()
    {

    }   
    
    public void periodic()
    {
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double y = driverController.getY(GenericHID.Hand.kLeft);
        if(Math.abs(y) < 0.15)
            y = 0.0;
        // final double xSpeed = -m_speedLimiter.calculate(y) * Drivetrain.kMaxSpeed;
        final double xSpeed = -y * Constants.kMaxSpeed;

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double x = driverController.getX(GenericHID.Hand.kRight);
        if(Math.abs(x) < 0.15)
            x = 0.0;
        // final double rot = -m_rotLimiter.calculate(x) * Drivetrain.kMaxAngularSpeed;
        final double rot = x * Constants.kMaxAngularSpeed;

        drivetrain.arcadeDrive(xSpeed / 2.0, rot / 2.0);

        // System.out.println(drivetrain);
    }
    
    public void end()
    {
        
    }
}
