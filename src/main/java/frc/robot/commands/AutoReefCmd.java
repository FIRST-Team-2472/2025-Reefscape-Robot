package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSequences;
import frc.robot.Constants;
import frc.robot.subsystems.CoralDispenserSubsytem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoReefCmd extends Command{
    CommandSequences commandSequences;
    CoralDispenserSubsytem coralDispenserSubsytem;
    ElevatorSubsystem elevatorSubsystem;
    SwerveSubsystem swerveSubsystem;
    int T, Q, P, reefLevel;     
    double X, Y; 
    double elevatorHeight;
    Pose2d reefLocation; 
    Command placeCoralOnReef;
    public AutoReefCmd(CommandSequences commandSequences, CoralDispenserSubsytem coralDispenserSubsytem, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, int reefLevel) {
        this.commandSequences = commandSequences;
        this.coralDispenserSubsytem = coralDispenserSubsytem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.reefLevel = reefLevel;
        addRequirements(coralDispenserSubsytem, elevatorSubsystem, swerveSubsystem);
    }

    @Override
    public void initialize() {      
        X = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
        Y = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
        X-= 4.5;
        Y-= 4.05;
        Q = findQ();
        T = findT();
        P = findP();
        if (reefLevel == 1){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL1Height;
            reefLocation = l1Trough();
        } else if (reefLevel == 2){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL2Height;
        } else if (reefLevel == 3){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL3Height;
        } else {
            elevatorHeight = Constants.ElevatorConstants.kElevatorL4Height;
        }
        placeCoralOnReef = commandSequences.placeCoralOnReef(swerveSubsystem, elevatorSubsystem, coralDispenserSubsytem, reefLocation, reefLevel);
        placeCoralOnReef.schedule();
}
    @Override
    public boolean isFinished() {
        return true;
    }

    private int findQ() {
        if (X >= 0) {
            if (Y >= 0) {
                return 1;
            } else {
                return 4;
            }
        } else if (Y >= 0 && X < 0) {
            return 2;
        } else {
            return 3;
        }
    }

    private int findT() {
        double m = 0.57735;
        if (Q == 2 || Q == 4){
            m = m *-1;
        }
        double lineY = 0.57735 * X;
        double lineX = 0.57735 * Y;
        if (Math.abs(Y) < Math.abs(lineY)){
            return 1;
        } else if (Math.abs(X) < Math.abs(lineX)) {
            return 3;
        } else {
            return 2;
        }
    }

    private int findP() {
        if (Q == 2){
            if (T == 3){
                reefLocation = Constants.ReefConstants.reefK;
                return 11;
            } else if (T == 2){
                reefLocation = Constants.ReefConstants.reefL;
                return 12;
            } else {
                reefLocation = Constants.ReefConstants.reefA;
                return 1;
            }
        } else if (Q == 3){
            if (T == 1){
                reefLocation = Constants.ReefConstants.reefB;
                return 2;
            } else if (T == 2){
                reefLocation = Constants.ReefConstants.reefC;
                return 3;
            } else {
                reefLocation = Constants.ReefConstants.reefD;
                return 4;
            }
        } else if (Q == 4){
            if (T == 3){
                reefLocation = Constants.ReefConstants.reefE;
                return 5;
            } else if (T == 2){
                reefLocation = Constants.ReefConstants.reefF;
                return 6;
            } else {
                reefLocation = Constants.ReefConstants.reefG;
                return 7;
            } 
        }else {
            if (T == 1){
                reefLocation = Constants.ReefConstants.reefH;
                return 8;
            } else if (T == 2){
                reefLocation = Constants.ReefConstants.reefI;
                return 9;
            } else {
                reefLocation = Constants.ReefConstants.reefJ;
                return 10;
            }
        }
    }

    private Pose2d l1Trough(){
        if (P == 1 || P == 2){
            return Constants.ReefConstants.reefTroughAB;
        } else if (P == 3 || P == 4){
            return Constants.ReefConstants.reefTroughCD;
        } else if (P == 5 || P == 6){
            return Constants.ReefConstants.reefTroughEF;
        } else if (P == 7 || P == 8){
            return Constants.ReefConstants.reefTroughGH;
        } else if (P == 9 || P == 10){
            return Constants.ReefConstants.reefTroughIJ;
        } else {
            return Constants.ReefConstants.reefTroughKL;
        }
    }
}
