package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Limelight limelight = new Limelight();

    public double[] LLPoseData;
    double targetAngle = 0;
    boolean aimControl = false;
    public double thetaPID = 0;

    public PIDController robotThetaController = new PIDController(0.01, 0.0, 0.0000001);

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.FrontLeftMod.constants),
            new SwerveModule(1, Constants.Swerve.FrontRightMod.constants),
            new SwerveModule(2, Constants.Swerve.BackLeftMod.constants),
            new SwerveModule(3, Constants.Swerve.BackRightMod.constants)
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());

        configAutoBuilder();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    } 
    

    public void configAutoBuilder(){
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::resetOdometry, 
            this::getModuleSpeeds, 
            this::driveRobotRelative, 
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0), 4.5, 0.4, new ReplanningConfig()), 
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this 
            );
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.2);

        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public ChassisSpeeds getModuleSpeeds(){
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getRoll(){
        return gyro.getRoll();
        
    }

    public String getError(){
        return "";
    }

    public double getAimValue(){

        double xGoalPos = 0, yGoalPos = 0;

        if(DriverStation.getAlliance().toString() == "blue"){
            xGoalPos = 0.0;
            yGoalPos = 5.55;
        }
        else{
            xGoalPos = 0.0;
            yGoalPos = 5.55;
        }

        targetAngle =  Math.toDegrees(Math.atan((this.getPose().getY() - yGoalPos) / (this.getPose().getX() - xGoalPos)));

        return targetAngle;

    }

    public void setAimControl(boolean aimControl){
        this.aimControl = aimControl;
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());
        if(limelight.validTarget()){
            LLPoseData = limelight.getLLPose(); 
            swerveOdometry.resetPosition(new Rotation2d(getYaw().getDegrees()), getModulePositions(), new Pose2d(new Translation2d(LLPoseData[0], LLPoseData[1]), new Rotation2d(LLPoseData[5])));
            //gyro.setYaw(LLPoseData[5]);
        }


        if(aimControl){
            thetaPID = robotThetaController.calculate(getYaw().getDegrees(), targetAngle);
        }
        

        SmartDashboard.putNumber("Robot PoseX", this.getPose().getX());
        SmartDashboard.putNumber("Robot PoseY", this.getPose().getY());
        SmartDashboard.putNumber("Robot PoseYaw", getYaw().getDegrees());

        SmartDashboard.putNumber("PID Controller Output", thetaPID);
        SmartDashboard.putNumber("Robot Aim Angle", getAimValue());
    }
}