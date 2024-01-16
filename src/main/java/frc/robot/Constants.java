package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double MAX_VOLTS = 12.0;

    //Motor ID's

    public static final int conveyorID = 15;
    public static final int intakeRollerTopID = 16;
    public static final int liftRotateID = 17;
    public static final int intakeExtendID = 18;
    public static final int liftExtendID = 19;
    public static final int grabberID = 20;
    public static final int intakeRollerBottomID = 21;

    //Sensor ID's

    public static final int photoEyeID1 = 0;
    public static final int photoEyeID2 = 1;
    public static final int liftExtendZeroID = 2;
    public static final int intakeZeroID = 4;
    public static final int liftRotateZeroID = 3;
    public static final int grabberZeroID = 5;

    //PWM

    public static final int leftLEDStripID = 9;

    //Speeds

    public static final double CONVEYOR_FORWARD_SPEED_VOLTS = MAX_VOLTS * 0.5;
    public static final double CONVEYOR_BACKWARD_SPEED_VOLTS = MAX_VOLTS * -0.5;
    public static final double INTAKE_EXTENTION_SPEED_VOLTS = MAX_VOLTS * 0.4;
    public static final double INTAKE_RETRACTION_SPEED_VOLTS = MAX_VOLTS * -0.4;
    public static final double INTAKE_ROLLER_SPEED_VOLTS = MAX_VOLTS * -0.35;
    public static final double LIFT_EXTEND_SPEED_VOLTS = MAX_VOLTS * -0.5;
    public static final double LIFT_ROTATE_SPEED_VOLTS = MAX_VOLTS * -0.5;

    //Motor Position
    public static final double INTAKE_EXTEND_POSITION = 55.0;
    public static final double INTAKE_RETRACT_POSITION = -4.0;

    public static final double LIFT_EXTEND_POSITION_0 = 0.0; //Conveyor Position
    public static final double LIFT_ROTATE_POSITION_0 = -2.0;
    public static final double LIFT_EXTEND_POSITION_1 = 0.0; //Level 1 Scoring Position
    public static final double LIFT_ROTATE_POSITION_1 = 50.0;
    public static final double LIFT_EXTEND_POSITION_2 = 30.0; //Level 2 Scoring Position
    public static final double LIFT_ROTATE_POSITION_2 = 112.0;
    public static final double LIFT_EXTEND_POSITION_3 = 130.0; //Level 3 Scoring Position
    public static final double LIFT_ROTATE_POSITION_3 = 130.0;
    public static final double LIFT_EXTEND_POSITION_INTAKE = 0.0; //Intake Position
    public static final double LIFT_ROTATE_POSITION_INTAKE = 110.0;
    public static final double LIFT_ROTATE_CLEAR_POSITION = 40.0; //Position where extension can proceed into the lowered position.
    public static final double LIFT_EXTEND_CLEAR_POSITION = 40.0; //Position where rotation can proceed into the lowered position.
    public static final double LIFT_ROTATE_POSITION_GRAB = 0.0;
    public static final double LIFT_EXTEND_POSITION_GRAB = 7.0;

    public static final double LIFT_SETPOINT_DROP_LIMIT = 90.0;
    public static final double LIFT_SETPOINT_DROP = 13.0;

    public static final double GRABBER_OPEN_POSITION = 0.0;
    public static final double GRABBER_CLOSED_POSITION = 32.0;

    public static final double GRABBER_LIFT_CLOSED_THRESHOLD = 2.0;

    public static final double LIFT_ROTATE_CURRENT_EMERGENCY_STOP = 0.0;
    public static final double LIFT_EXTEND_CURRENT_EMERGENCY_STOP = 0.0;

    //Servo Positions

    public static final double CONVEYOR_SERVO_HIGH_POSITION = 70.0;
    public static final double CONVEYOR_SERVO_LOW_POSITION = 40.0;
    

    //Ramp Rates

    public static final double CONVEYOR_RAMP_RATE = 0.1;
    public static final double INTAKE_EXTEND_RAMP_RATE = 0.25;
    public static final double INTAKE_ROLLER_RAMP_RATE = 0.1;

    //Software Current Limits

    public static final double CONVEYOR_MAX_CURRENT = 40.0;
    public static final double INTAKE_EXTEND_MAX_CURRENT = 40.0;
    public static final double INTAKE_ROLLER_MAX_CURRENT = 40.0;

    //Software Limits
    public static final float INTAKE_FORWARD_LIMIT = 58.0f;
    public static final float INTAKE_REVERSE_LIMIT = -4.0f;

    public static final float GRABBER_FORWARD_LIMIT = 34.0f;
    public static final float GRABBER_REVERSE_LIMIT = 0.0f;
    
    public static final float LIFT_EXTEND_FORWARD_LIMIT = 150.0f;
    public static final float LIFT_EXTEND_REVERSE_LIMIT = -30.0f;
    public static final float LIFT_ROTATE_FORWARD_LIMIT = 175.0f;
    public static final float LIFT_ROTATE_REVERSE_LIMIT = -30.0f;
    //public static final float INTAKE_EXTEND_FORWARD_LIMIT = 46.0f;
    //public static final float INTAKE_EXTEND_REVERSE_LIMIT = 0.0f;

    //Timers

    public static final double INTAKE_DELAY_TIMER = 1.0;
    public static final double CONVEYOR_DELAY_TIMER = 1.0;

    //Limelight

    public static final double LIMELIGHT_TX_OFFSET = -12.0;
    public static final double LIMELIGHT_STRAFE_ERROR_MARGIN = 0.25;

    //Sonic Sensor Targeting

    public static final double SONIC_TARGET_DISTANCE = 2.0;
    public static final double SONIC_TARGETING_DRIVE_SPEED = -0.25;
    public static final double SONIC_TARGETING_ERROR_MARGIN = 0.25;

    //Shuffleboard Tabs

    public static final ShuffleboardTab matchTab = Shuffleboard.getTab("Match");
    public static final ShuffleboardTab swerveDebugTab = Shuffleboard.getTab("Swerve Debug");
    public static final ShuffleboardTab conveyorDebugTab = Shuffleboard.getTab("Conveyor Debug");
    public static final ShuffleboardTab intakeDebugTab = Shuffleboard.getTab("Intake Debug");
    public static final ShuffleboardTab liftDebugTab = Shuffleboard.getTab("Lift Debug");
    public static final ShuffleboardTab grabberDebugTab = Shuffleboard.getTab("Grabber Debug");
    public static final ShuffleboardTab limelightDebugTab = Shuffleboard.getTab("Limelight Debug");
    public static final ShuffleboardTab pidConfigTab = Shuffleboard.getTab("PID Config");


    public static final class Sensors{
        public static DigitalInput photoeye1 = new DigitalInput(Constants.photoEyeID1);
        public static DigitalInput photoeye2 = new DigitalInput(Constants.photoEyeID2);
        public static DigitalInput intakeZero = new DigitalInput(Constants.intakeZeroID);
        public static DigitalInput liftRotateZero = new DigitalInput(Constants.liftRotateZeroID);
        public static DigitalInput liftExtendZero = new DigitalInput(Constants.liftExtendZeroID);
        public static DigitalInput grabberZero = new DigitalInput(Constants.grabberZeroID);
    }

    public static final class Swerve {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(23.5); 
        public static final double wheelBase = Units.inchesToMeters(20.25); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12); //TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; //TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class FrontLeftMod { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-148 + 180 - 110); //273.3
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class FrontRightMod { 
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(20.16 - 110);   //3.7
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class BackLeftMod { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-50.08 - 110); //329.7
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class BackRightMod { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(357 - 110); //258.5
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}