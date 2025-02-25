package frc.robot.Commands.DriveCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Vision.PhotonVisionCamera;

public class AutoAlignToSourceCommand extends Command {

    SwerveSubsystem swerve;
    PhotonVisionCamera camera;

    private double tagYaw;
    private double tagPitch;

    private double speed = 0.3;

    public AutoAlignToSourceCommand(SwerveSubsystem swerve, PhotonVisionCamera camera){
        this.swerve = swerve;
        this.camera = camera;
        addRequirements(swerve);
    }   

    private double getXtranslation() {
        if (tagPitch < -4) return -speed;
        if (tagPitch > -3) return speed;
        return 0;
      }
    
      private double getYtranslation() {
        if (tagYaw < -2) return -speed;
        if (tagYaw > 2) return speed;
        return 0;
      }
    
      private double getRotation() {
        if (tagYaw > 1) return -speed;
        if (tagYaw < -1) return speed; 
        return 0;
      }
    
      private void getTagData() {
        tagYaw = camera.getTargetYaw();
        tagPitch = camera.getTargetPitch();
      }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        swerve.drive(new Translation2d(getXtranslation(), getYtranslation()), getRotation(), false);
    }

    @Override
    public boolean isFinished(){
        getTagData();
    if (!camera.hasTarget()) {
      System.out.println("No Target");
      return true;
    }
    return (getXtranslation() == 0 && getYtranslation() == 0 && getRotation() == 0);
    }

    @Override
    public void end(boolean in){

    }
    
}
