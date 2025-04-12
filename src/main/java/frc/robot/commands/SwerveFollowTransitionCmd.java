package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.extras.FieldPose2d;
import frc.robot.extras.PosPose2d;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveFollowTransitionCmd extends Command {
    SwerveSubsystem swerveSubsystem;
    FieldPose2d startPose, endPose, targetPose;
    double xTransitionPerFrame, yTransitionPerFrame, angleTransitionPerFrame;

    /**
     * @param swerveSubsystem the swerve subsystem
     * @param startPose PosPose2d of the startPose
     * @param endPose the pose to transition to and end at
     * @param transitionTime the time it should take to fully transition to the end pose
     */
    public SwerveFollowTransitionCmd(SwerveSubsystem swerveSubsystem, PosPose2d startPose, PosPose2d endPose, double transitionTime){
        addRequirements(swerveSubsystem);

        this.swerveSubsystem = swerveSubsystem;
        this.startPose = startPose.toFieldPose2d();
        this.endPose = endPose.toFieldPose2d();
        targetPose = this.startPose;
        // need to use the global one so its converted
        xTransitionPerFrame = (this.endPose.getX()-this.startPose.getX())/50/transitionTime;// 50 is code refreshes per second
        yTransitionPerFrame = (this.endPose.getY()-this.startPose.getY())/50/transitionTime;
        angleTransitionPerFrame = this.endPose.getRotation().minus(this.startPose.getRotation()).getDegrees()/50/transitionTime;
    }

    @Override
    public void initialize() {
        swerveSubsystem.initializeDriveToPointAndRotate(startPose);
    }

    @Override
    public void execute() {
        calculateCurrentPose();
        swerveSubsystem.executeDriveToPointAndRotate(targetPose);
    }

    public void calculateCurrentPose(){
        //if we are at the end pose we will just return
        //the < .01 is because doubles rarely exactly equal eachother
        if(Math.abs(targetPose.getX() - endPose.getX()) < .01 && Math.abs(targetPose.getY() - endPose.getY()) < .01  && Math.abs(targetPose.getRotation().minus(endPose.getRotation()).getDegrees())  < .01)
            return;
        //creating a new pose by adding the transition per frame to the old one
        targetPose = new FieldPose2d(targetPose.getX() + xTransitionPerFrame, targetPose.getY() + yTransitionPerFrame, Rotation2d.fromDegrees(targetPose.getRotation().getDegrees() + angleTransitionPerFrame));
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        // use this function if you overide the command to finsih it
        return swerveSubsystem.isAtPoint(endPose.getTranslation()) && swerveSubsystem.isAtAngle(endPose.getRotation());
    }
}
