//////////// NAVX GYRO SUBSYSTEM ////////////

// the NavX software (class AHRS) provides a complete source of information
// that no new methods are here except a couple of display values methods
// handy for debugging

// checking for AHRS instantiation and calibration down time used in past years
// could be added here

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;

public class NavX extends AHRS
{
    public NavX(SerialPort.Port serial_port_id, SerialDataType data_type, byte update_rate_hz) 
    {
        super(serial_port_id, data_type, update_rate_hz);
        init();
    }

    private void init() 
    {
        try 
        {
            Timer.delay(1.0); // make sure AHRS USB communication is done before doing
            // other USB. Also give it time to settle in; a few more seconds never hurt
            enableLogging(true);
            System.out.println(getActualUpdateRate() + " " + getUpdateCount() + " " + getRequestedUpdateRate());
        }
        catch (final RuntimeException ex)
        {
            DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
        }

        reset();
    }
      
    public void displayOnSmartDashboard()
    {
        // fused heading 0 to 360
        // IMU total yaw -inf to +inf
        // IMU yaw -180 to +180
        // IMU yaw rate dps -big to +big
        SmartDashboard.putNumber("IMU_Yaw", getYaw());
        SmartDashboard.putNumber("IMU_Pitch", getPitch());
        SmartDashboard.putNumber("IMU_Roll", getRoll());
    
        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */
        SmartDashboard.putBoolean("Magnetic Disturbance", isMagneticDisturbance());
        SmartDashboard.putBoolean("Magnetometer Calibrated", isMagnetometerCalibrated());
    
        SmartDashboard.putNumber("IMU_CompassHeading", getCompassHeading());
    
        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        SmartDashboard.putNumber("IMU_FusedHeading", getFusedHeading());
    
        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */
        SmartDashboard.putNumber("IMU_TotalYaw", getAngle());
        SmartDashboard.putNumber("IMU_YawRateDPS", getRate());
    
        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */
        SmartDashboard.putNumber("IMU_Accel_X", getWorldLinearAccelX());
        SmartDashboard.putNumber("IMU_Accel_Y", getWorldLinearAccelY());
        SmartDashboard.putNumber("IMU_Accel_Z", getWorldLinearAccelZ());
        SmartDashboard.putBoolean("IMU_IsMoving", isMoving());
        SmartDashboard.putBoolean("IMU_IsRotating", isRotating());
    
        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */
        SmartDashboard.putNumber("Velocity_X", getVelocityX());
        SmartDashboard.putNumber("Velocity_Y", getVelocityY());
        SmartDashboard.putNumber("Velocity_Z", getVelocityZ());
        SmartDashboard.putNumber("Displacement_X", getDisplacementX());
        SmartDashboard.putNumber("Displacement_Y", getDisplacementY());
        SmartDashboard.putNumber("Displacement_Z", getDisplacementZ());
    
        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */
        SmartDashboard.putNumber("RawGyro_X", getRawGyroX());
        SmartDashboard.putNumber("RawGyro_Y", getRawGyroY());
        SmartDashboard.putNumber("RawGyro_Z", getRawGyroZ());
        SmartDashboard.putNumber("RawAccel_X", getRawAccelX());
        SmartDashboard.putNumber("RawAccel_Y", getRawAccelY());
        SmartDashboard.putNumber("RawAccel_Z", getRawAccelZ());
        SmartDashboard.putNumber("RawMag_X", getRawMagX());
        SmartDashboard.putNumber("RawMag_Y", getRawMagY());
        SmartDashboard.putNumber("RawMag_Z", getRawMagZ());
        SmartDashboard.putNumber("IMU_Temp_C", getTempC());
        SmartDashboard.putNumber("IMU_Timestamp", getLastSensorTimestamp());
    
        SmartDashboard.putNumber("Fused Heading", getFusedHeading());
    
        /* Omnimount Yaw Axis Information */
        /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        final AHRS.BoardYawAxis yaw_axis = getBoardYawAxis();
        SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
        // System.out.println(AHRS.BoardAxis.kBoardAxisX + " " +
        // AHRS.BoardAxis.kBoardAxisY + " " + AHRS.BoardAxis.kBoardAxisZ +
        // yaw_axis.board_axis.getValue());
        final String kBoardAxisAlpha[] = { "BoardAxisX", "BoardAxisY", "BoardAxisZ" };
        SmartDashboard.putString("YawAxisAlpha", kBoardAxisAlpha[yaw_axis.board_axis.getValue()]);
    
        /* Sensor Board Information */
        SmartDashboard.putString("FirmwareVersion", getFirmwareVersion());
    
        /* Quaternion Data */
        /* Quaternions are fascinating, and are the most compact representation of */
        /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        /* from the Quaternions. If interested in motion processing, knowledge of */
        /* Quaternions is highly recommended. */
        SmartDashboard.putNumber("QuaternionW", getQuaternionW());
        SmartDashboard.putNumber("QuaternionX", getQuaternionX());
        SmartDashboard.putNumber("QuaternionY", getQuaternionY());
        SmartDashboard.putNumber("QuaternionZ", getQuaternionZ());
    
        /* Connectivity Debugging Support */
        SmartDashboard.putNumber("IMU_Byte_Count", getByteCount());
        SmartDashboard.putNumber("IMU_Update_Count", getUpdateCount());
        // if (ahrs->IsAltitudeValid()) // Aero only
        // {
        // SmartDashboard::PutNumber( "Barometric Pressure",
        // ahrs->GetBarometricPressure() );
        // SmartDashboard::PutNumber( "Altitude", ahrs->GetAltitude() );
        // SmartDashboard::PutNumber( "Pressure", ahrs->GetPressure() );
        // }
        // else
        // {
        // SmartDashboard::PutString( "Barometric Pressure", (llvm::StringRef)"Not
        // Available" );
        // SmartDashboard::PutString( "Altitude", (llvm::StringRef)"Not Available" );
        // SmartDashboard::PutString( "Pressure", (llvm::StringRef)"Not Available" );
        // }
    
        /* Display 6-axis Processed Angle Data */
        SmartDashboard.putBoolean("IMU_Connected", isConnected());
        SmartDashboard.putBoolean("IMU_IsCalibrating", isCalibrating());
    }

    public String toString() 
    {
        return "angle:" + getAngle() + ", rate:" + getRate() + ", " + getRotation2d();
    }

}

/***********************************************************************
 * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
 * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
 * 
 * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
 * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
 * 
 * Multiple navX-model devices on a single robot are supported.
 ************************************************************************/
// ahrs = new AHRS(SerialPort.Port.kUSB1);
// ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 200);
// ahrs = new AHRS(SPI.Port.kMXP);
// ahrs = new AHRS(I2C.Port.kMXP);
// ahrs = new AHRS(I2C.Port.kOnboard, (byte) 60);


// ** Interface for yaw rate gyros. */
// public interface Gyro extends AutoCloseable {
//     /**
//      * Calibrate the gyro. It's important to make sure that the robot is not moving while the
//      * calibration is in progress, this is typically done when the robot is first turned on while it's
//      * sitting at rest before the match starts.
//      */
//     void calibrate();
  
//     /**
//      * Reset the gyro. Resets the gyro to a heading of zero. This can be used if there is significant
//      * drift in the gyro and it needs to be recalibrated after it has been running.
//      */
//     void reset();
  
//     /**
//      * Return the heading of the robot in degrees.
//      *
//      * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
//      * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
//      * 360 to 0 on the second time around.
//      *
//      * <p>The angle is expected to increase as the gyro turns clockwise when looked at from the top.
//      * It needs to follow the NED axis convention.
//      *
//      * <p>This heading is based on integration of the returned rate from the gyro.
//      *
//      * @return the current heading of the robot in degrees.
//      */
//     double getAngle();
  
//     /**
//      * Return the rate of rotation of the gyro.
//      *
//      * <p>The rate is based on the most recent reading of the gyro analog value
//      *
//      * <p>The rate is expected to be positive as the gyro turns clockwise when looked at from the top.
//      * It needs to follow the NED axis convention.
//      *
//      * @return the current rate in degrees per second
//      */
//     double getRate();
  
//     /**
//      * Return the heading of the robot as a {@link edu.wpi.first.wpilibj.geometry.Rotation2d}.
//      *
//      * <p>The angle is continuous, that is it will continue from 360 to 361 degrees. This allows
//      * algorithms that wouldn't want to see a discontinuity in the gyro output as it sweeps past from
//      * 360 to 0 on the second time around.
//      *
//      * <p>The angle is expected to increase as the gyro turns counterclockwise when looked at from the
//      * top. It needs to follow the NWU axis convention.
//      *
//      * <p>This heading is based on integration of the returned rate from the gyro.
//      *
//      * @return the current heading of the robot as a {@link
//      *     edu.wpi.first.wpilibj.geometry.Rotation2d}.
//      */
//     default Rotation2d getRotation2d() {
//       return Rotation2d.fromDegrees(-getAngle());
//     }
//   }
