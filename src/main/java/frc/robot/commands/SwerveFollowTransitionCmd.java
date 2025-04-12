package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.FieldPose2d;
import frc.robot.extras.PosPose2d;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveFollowTransitionCmd extends Command {
    SwerveSubsystem swerveSubsystem;
    PosPose2d startPose, endPose, targetPose;
    FieldPose2d drivePose;
    double xTransitionPerFrame, yTransitionPerFrame, angleTransitionPerFrame;

    Timer timer = new Timer();

    /**
     * @param swerveSubsystem the swerve subsystem
     * @param startPose PosPose2d of the startPose
     * @param endPose the pose to transition to and end at
     * @param transitionTime the time it should take to fully transition to the end pose
     */
    public SwerveFollowTransitionCmd(SwerveSubsystem swerveSubsystem, PosPose2d startPose, PosPose2d endPose, double transitionTime){
        addRequirements(swerveSubsystem);

        this.swerveSubsystem = swerveSubsystem;
        this.startPose = startPose;
        this.endPose = endPose;
        targetPose = this.startPose;

        xTransitionPerFrame = (endPose.getX()-startPose.getX())/50/transitionTime;// 50 is code refreshes per second
        yTransitionPerFrame = (endPose.getY()-startPose.getY())/50/transitionTime;
        angleTransitionPerFrame = endPose.getRotation().minus(startPose.getRotation()).getDegrees()/50/transitionTime;
    }

    @Override
    public void initialize() {
        timer.restart();
        swerveSubsystem.initializeDriveToPointAndRotate(startPose);
    }

    @Override
    public void execute() {
        calculateCurrentPose();
        swerveSubsystem.executeDriveToPointAndRotate(drivePose);
    }

    public void calculateCurrentPose(){
        //if we are at the end pose we will just return
        //the < .01 is because doubles rarely exactly equal eachother
        if(Math.abs(targetPose.getX() - endPose.getX()) < .01 && Math.abs(targetPose.getY() - endPose.getY()) < .01  && Math.abs(targetPose.getRotation().minus(endPose.getRotation()).getDegrees())  < .01){
            targetPose = endPose;
            return;
        }
            
        //creating a new pose by adding the transition per frame to the old one
        targetPose = new PosPose2d(targetPose.getX() + xTransitionPerFrame, targetPose.getY() + yTransitionPerFrame, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees() + angleTransitionPerFrame));
        drivePose = targetPose.toFieldPose2d();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // use this function if you overide the command to finsih it
        if ((swerveSubsystem.isExactlyInPosition(endPose) || (swerveSubsystem.isNearlyInPosition(endPose) && swerveSubsystem.isAtAngle(endPose.getRotation()))) || timer.hasElapsed(3.5)){
        return true;
        }
        return false;    
    }
}
