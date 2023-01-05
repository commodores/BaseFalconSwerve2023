package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 13;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(18.6875);
        public static final double wheelBase = Units.inchesToMeters(18.6875);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = 6.75; //6.75:1
        public static final double angleGearRatio = (150.0 / 7.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;//25
        public static final int anglePeakCurrentLimit = 20;//40
        public static final double anglePeakCurrentDuration = 0.1;//.1
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 25;//35
        public static final int drivePeakCurrentLimit = 25;//60
        public static final double drivePeakCurrentDuration = 0;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = .15;//0.12;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.01;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.05;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxVelocity = ((6380 / 60) / driveGearRatio) * wheelDiameter * Math.PI * 0.95; // meters per second
        //public static final double maxAccel = maxVelocity * 1.5; // take 1/2 sec to get to max speed.
        public static final double maxAngularVelocity = maxVelocity / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        //public static final double maxAngularAcceleration = Math.pow(maxAngularVelocity, 2);

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = true;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 1;
            public static final double angleOffset = 292.060;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 2;
            public static final double angleOffset = 78.222;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 4;
            public static final double angleOffset = 32.871;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 9;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 3;
            public static final double angleOffset = 299.443;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;//Swerve.maxAngularVelocity;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;//Swerve.maxAngularVelocity;
    
        public static final double kPThetaController = 4; 
        public static final double kPXandYControllers = 2; 
        
    }

    public static final class VisionConstants {
        public static final double kCameraHeight = 2.75; // Camera height in feet
        public static final double kTargetHeight = 8.666667; // Target height in feet
        public static final double kCameraToTarget = kTargetHeight - kCameraHeight; // height difference between the target and camera
        public static final double kCameraMountAngle = 39.5; // Angle in degrees from the horizon
    }

}
