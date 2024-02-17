package frc.robot.commands.Swerve;

import frc.robot.Constants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;


public class targetAim extends Command {    
    private Swerve s_Swerve;
    private PhotonVision p_PhotonVision;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public targetAim(Swerve s_Swerve, PhotonVision p_PhotonVision, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        this.p_PhotonVision = p_PhotonVision;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        if(p_PhotonVision.getHasTarget()){
          s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            p_PhotonVision.getPIDRotateValue() * Constants.Swerve.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean(), 
            true
          );
        }
        else{
          s_Swerve.drive(
              new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
              rotationVal * Constants.Swerve.maxAngularVelocity, 
              !robotCentricSup.getAsBoolean(), 
              true
          );
        }
    }
}