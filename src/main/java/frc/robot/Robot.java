// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.util.Units;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class Robot extends TimedRobot 
{
    private final XboxController m_controller = new XboxController(0);
    private final Drivetrain m_drive = new Drivetrain();

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    // private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(3);
    // private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // An example trajectory to follow during the autonomous period.
    private Trajectory m_trajectory[] = new Trajectory[4];

    // The Ramsete Controller to follow the trajectory.
    private RamseteController m_ramseteController = new RamseteController(2.0, 0.7);

    // The timer to use during the autonomous period.
    private Timer m_timer;

    private int m_path = 1;

    @Override
    public void robotInit() 
    {
        // Create the trajectory to follow in autonomous. It is best to initialize
        // trajectories here to avoid wasting time in autonomous.
        
        // ORIGINAL
        // m_trajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //         List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        //         new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        //         new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

        // m_trajectory =
        // TrajectoryGenerator.generateTrajectory(
        //     new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        //     List.of(new Translation2d(1, 0), new Translation2d(2, 0)),
        //     new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
        //     m_drive.getTrajectoryConfig());

        for(int i = 0; i < 4; i++) //TODO use m_trajectory.length
        {
            String trajectoryJSON = "output/Path 0" + (i+1) + ".wpilib.json";
            try
            {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                m_trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

                //System.out.println(autoTrajectory[0]);
            }
            catch (IOException ex)
            {
                DriverStation.reportError("Unable to open trajectory : " + trajectoryJSON, ex.getStackTrace());
            }
        }
    }

    @Override
    public void autonomousInit() 
    {
        m_path = 1;
        // Initialize the timer.
        m_timer = new Timer();
        m_timer.start();

        // Reset the drivetrain's odometry to the starting pose of the trajectory.
        m_drive.resetOdometry(m_trajectory[m_path-1].getInitialPose());
    }

    @Override
    public void autonomousPeriodic() 
    { //TODO have a start-of-path-segment switch and move what's in the "else" block to here plus start.. = false
        // Update odometry.
        m_drive.updateOdometry();
        System.out.println(m_drive);

        if (m_path <= 4 && m_timer.get() < m_trajectory[m_path-1].getTotalTimeSeconds()) 
        {
            // Get the desired pose from the trajectory.
            State desiredPose = m_trajectory[m_path-1].sample(m_timer.get());
            // System.out.println(m_drive.getPose());

            // Get the reference chassis speeds from the Ramsete controller.
            ChassisSpeeds refChassisSpeeds = m_ramseteController.calculate(m_drive.getPose(), desiredPose);

            // Set the linear and angular speeds.
            m_drive.drive(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
        } 
        else 
        { //TODO set start ... = true and move these 3 lines to beginning
            m_drive.drive(0, 0); // make a method to stop all motors.  For the SparkMax they prefer you use their stopMotor().  Talon might not have that so make our own.
            m_path++;
            m_timer.reset();
        }
    }

    @Override
    public void teleopPeriodic() 
    {

        System.out.println(m_drive);
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        double y = m_controller.getY(GenericHID.Hand.kLeft) / 2.0;
        if(Math.abs(y) < 0.2)
            y = 0.0;
        // final double xSpeed = -m_speedLimiter.calculate(y) * Drivetrain.kMaxSpeed;
        final double xSpeed = -y * Drivetrain.kMaxSpeed;
        // System.out.print("y= " + y + "  speed= " + xSpeed);

        // Get the rate of angular rotation. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        double x = m_controller.getX(GenericHID.Hand.kRight) / 2.0;
        if(Math.abs(x) < 0.2)
            x = 0.0;
        // final double rot = -m_rotLimiter.calculate(x) * Drivetrain.kMaxAngularSpeed;
        final double rot = -x * Drivetrain.kMaxAngularSpeed;
        // System.out.print("x= " + y + "  rot= " + rot + "\n");

        m_drive.drive(xSpeed, rot);
    }
}
