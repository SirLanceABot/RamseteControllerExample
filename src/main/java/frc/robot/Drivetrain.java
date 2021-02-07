// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS.SerialDataType;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import edu.wpi.first.wpilibj.SerialPort;

// import edu.wpi.first.wpilibj.AnalogGyro;
// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.PWMVictorSPX;
// import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

/** Represents a differential drive style drivetrain. */
public class Drivetrain 
{
    public static final double kMaxSpeed = 3.0; // meters per second
    public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

    private static final double kTrackWidth = 0.7112; // meters outside wheel to outside wheel
    // private static final double kWheelRadius = 0.0508; // meters
    // private static final int kEncoderResolution = 4096;

    // private final SpeedController m_leftLeader = new PWMVictorSPX(1);
    // private final SpeedController m_leftFollower = new PWMVictorSPX(2);
    // private final SpeedController m_rightLeader = new PWMVictorSPX(3);
    // private final SpeedController m_rightFollower = new PWMVictorSPX(4);
    private static final CANSparkMax m_leftLeader = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    private static final CANSparkMax m_rightLeader = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    // private final Encoder m_leftEncoder = new Encoder(0, 1);
    // private final Encoder m_rightEncoder = new Encoder(2, 3);
    private static final CANEncoder m_leftEncoder = m_leftLeader.getEncoder();
    private static final CANEncoder m_rightEncoder = m_rightLeader.getEncoder();

    // private final SpeedControllerGroup m_leftGroup =
    //     new SpeedControllerGroup(m_leftLeader /*, m_leftFollower*/);
    // private final SpeedControllerGroup m_rightGroup =
    //     new SpeedControllerGroup(m_rightLeader /*, m_rightFollower*/);
    // private static final SpeedControllerGroup m_leftGroup = new SpeedControllerGroup(m_leftLeader);
    // private static final SpeedControllerGroup m_rightGroup = new SpeedControllerGroup(m_rightLeader);

    // private final AnalogGyro m_gyro = new AnalogGyro(0);
    private static final NavX m_gyro = new NavX(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 100);

    private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
    // private static final CANPIDController m_leftPIDController = m_rightLeader.getPIDController();
    // private static final CANPIDController m_rightPIDController = m_leftLeader.getPIDController();

    private static final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader, m_rightLeader);
    private static final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
    private final DifferentialDriveOdometry m_odometry;

    // Gains are for example purposes only - must be determined for your own robot!
    // private static final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private static final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.22, 1.98, 0.2);
    private static final DifferentialDriveVoltageConstraint m_autoVoltageConstraint =
                new DifferentialDriveVoltageConstraint(m_feedforward, m_kinematics, 6.0);
    private static final TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(1.0, 1.0).setKinematics(m_kinematics).addConstraint(m_autoVoltageConstraint);

    /**
     * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
     * gyro.
     */
    public Drivetrain() 
    {
        m_gyro.reset();

        m_leftLeader.restoreFactoryDefaults();
        m_leftLeader.setSmartCurrentLimit(Constants.NEO_SMART_CURRENT_LIMIT);
        m_leftLeader.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
        m_leftLeader.setIdleMode(IdleMode.kBrake);
        m_leftLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_leftLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_leftLeader.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
        m_leftLeader.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);

        m_rightLeader.restoreFactoryDefaults();
        m_rightLeader.setInverted(true); // right motor mounted mirror image - inverted - from the left
        m_rightLeader.setSmartCurrentLimit(Constants.NEO_SMART_CURRENT_LIMIT);
        m_rightLeader.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
        m_rightLeader.setIdleMode(IdleMode.kBrake);
        m_rightLeader.enableSoftLimit(SoftLimitDirection.kForward, false);
        m_rightLeader.enableSoftLimit(SoftLimitDirection.kReverse, false);
        m_rightLeader.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
        m_rightLeader.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen).enableLimitSwitch(false);
        m_drive.setRightSideInverted(false);

        // Set the distance per pulse for the drive encoders. We can simply use the
        // distance traveled for one rotation of the wheel divided by the encoder
        // resolution.
        // m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
        // m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

        // m_leftEncoder.reset();
        // m_rightEncoder.reset();

        m_leftEncoder.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV / 60.0);
        m_rightEncoder.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV / 60.0);

        m_leftEncoder.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV);
        m_rightEncoder.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV);

        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);

        m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    }

    /**
     * Sets the desired wheel speeds.
     *
     * @param speeds The desired wheel speeds.
     */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds)
    {
        final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

        final double leftOutput = m_leftPIDController.calculate(m_leftEncoder.getVelocity(), speeds.leftMetersPerSecond);
        final double rightOutput = m_rightPIDController.calculate(m_rightEncoder.getVelocity(), speeds.rightMetersPerSecond);

        // m_leftGroup.setVoltage(leftOutput + leftFeedforward);
        // m_rightGroup.setVoltage(rightOutput + rightFeedforward);
        m_leftLeader.setVoltage(leftOutput + leftFeedforward);
        m_rightLeader.setVoltage(rightOutput + rightFeedforward);
    }

    /**
     * Drives the robot with the given linear velocity and angular velocity.
     *
     * @param xSpeed Linear velocity in m/s.
     * @param rot Angular velocity in rad/s.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double rot)
    {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
        setSpeeds(wheelSpeeds);
        m_drive.feed();
    }

    /** Updates the field-relative position. */
    public void updateOdometry()
    {
        // m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
        m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
    }

    /**
     * Resets the field-relative position to a specific location.
     *
     * @param pose The position to reset to.
     */
    public void resetOdometry(Pose2d pose)
    {
        //resetEncoders();
        m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    }

    public void resetEncoders()
    {
        m_leftEncoder.setPosition(0.0);
        m_rightEncoder.setPosition(0.0);
    }

    /**
     * Returns the pose of the robot.
     *
     * @return The pose of the robot.
     */
    public Pose2d getPose()
    {
        return m_odometry.getPoseMeters();
    }

    public TrajectoryConfig getTrajectoryConfig()
    {
        return m_trajectoryConfig;
    }

    @Override
    public String toString()
    {
        String str;
        // str =
        //     String.format("LV: %.2f, RV: %.2f LP: %.2f, RP %.2f Yaw: %.2f",
        //     m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity(),
        //     m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
        //     m_gyro.getAngle());

        str =
            String.format("LP: %.2f, RP %.2f Yaw: %.2f Pose: %s",
            m_leftEncoder.getPosition(), m_rightEncoder.getPosition(),
            m_gyro.getAngle(), getPose());


        return str;
    }
}
