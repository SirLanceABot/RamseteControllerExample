package frc.robot;

public class Constants
{
    public static final int FRONT_LEFT_MOTOR_PORT = 4;
    public static final int FRONT_RIGHT_MOTOR_PORT = 1;
    public static final int BACK_RIGHT_MOTOR_PORT = 2;
    public static final int BACK_LEFT_MOTOR_PORT = 3;

    // public static final int PRIMARY_MOTOR_CURRENT_LIMIT = 35;
    // public static final int SECONDARY_MOTOR_CURRENT_LIMIT = 45;

    public static final double DRIVE_RAMP_TIME = 0.10;

    // public static final double MOTOR_DEADBAND = 0.01;

    // public static final double STARTING_SPEED = 0.3;
    // public static final double STOPPING_SPEED = 0.175;
    // public static final int ROTATE_THRESHOLD = 10;

    // public static final int LEFT_ENCODER_CHANNEL_A = 18;
    // public static final int LEFT_ENCODER_CHANNEL_B = 16;
    // public static final int RIGHT_ENCODER_CHANNEL_A = 14;
    // public static final int RIGHT_ENCODER_CHANNEL_B = 15;

    // 4096 ticks per motor revolution native NEO brushless
    //public static final double ENCODER_TICKS_PER_INCH = (360.0 * 4.0) / (3.25 * Math.PI);
    public static final double ENCODER_METER_PER_REV = 1.0 / 19.05; // approximately
    // public static final double ENCODER_METER_PER_TICK = ENCODER_METER_PER_REV / 4096.0;

    public static final int NEO_MOTOR_STALL_CURRENT = 150;
    public static final double NEO_MOTOR_FREE_CURRENT = 1.8;
    public static final int NEO_SMART_CURRENT_LIMIT = (int) (0.5 * (NEO_MOTOR_STALL_CURRENT - NEO_MOTOR_FREE_CURRENT) + NEO_MOTOR_FREE_CURRENT);
}