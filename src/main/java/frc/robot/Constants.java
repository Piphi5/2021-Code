// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DrivetrainConstants {
    public static final int kLeftSRXPort = 0;
    public static final int kRightSRXPort = 1;
    public static final int kLeftSPXPort = 2;
    public static final int kRightSPXPort = 3;

    public static final int kPigeonPort = 1;

    public static final int kCountsPerRev = 4096;  //Encoder counts per revolution of the motor shaft.
    public static final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels.  On the AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
    public static final double kGearRatio = 10.71; //Switch kSensorGearRatio to this gear ratio if encoder is on the motor instead of on the gearbox.
    public static final double kWheelRadiusInches = 3;
    public static final int k100msPerSecond = 10;

    public static final double kS = 0.22;
    public static final double kV = 1.98;
    public static final double kA = 0.2;

    public static final double kP = 8.5;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(DrivetrainConstants.kTrackwidthMeters);

  
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;


  }

  public final class ControllerConstants {
    public static final int kNavigatorPort = 0;
    
  }
  

}
