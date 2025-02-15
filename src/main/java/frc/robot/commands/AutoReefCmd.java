package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSequences;
import frc.robot.Constants;
import frc.robot.extras.PosPose2d;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoReefCmd extends Command{
    CommandSequences commandSequences;
    CoralDispenserSubsystem coralDispenserSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    SwerveSubsystem swerveSubsystem;
    int T, Q, P, reefLevel;     
    double X, Y; 
    double elevatorHeight;
    PosPose2d reefLocation; 
    Command placeCoralOnReef;
    boolean calculate;
    public AutoReefCmd(CommandSequences commandSequences, CoralDispenserSubsystem coralDispenserSubsystem, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, int reefLevel) {
        this.commandSequences = commandSequences;
        this.coralDispenserSubsystem = coralDispenserSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        this.reefLevel = reefLevel;
        calculate = true;
        addRequirements(coralDispenserSubsystem, elevatorSubsystem, swerveSubsystem);
    }
    public AutoReefCmd(CommandSequences commandSequences, CoralDispenserSubsystem coralDispenserSubsystem, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem, int reefLevel, PosPose2d reefLocation){
        this(commandSequences, coralDispenserSubsystem, elevatorSubsystem, swerveSubsystem, reefLevel);
        this.reefLocation = reefLocation;
        calculate = false;
    }

    @Override
    public void initialize() {      
        if (calculate){
            X = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
            Y = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
            X-= 4.5;// reef x center
            Y-= 4.05;// reef y center
            Q = findQ();
            T = findT();
            P = findP(); // also sets the reefLocation
        } 
        if (reefLevel == 1){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL1Height;
            if (calculate){
                reefLocation = l1Trough();
            }
        } else if (reefLevel == 2){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL2Height;
        } else if (reefLevel == 3){
            elevatorHeight = Constants.ElevatorConstants.kElevatorL3Height;
        } else {
            elevatorHeight = Constants.ElevatorConstants.kElevatorL4Height;
        }
        placeCoralOnReef = commandSequences.placeCoralOnReef(swerveSubsystem, elevatorSubsystem, coralDispenserSubsystem, reefLocation, elevatorHeight, calculate);
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
        double m = 0.57735;// slope of 30 degree line
        if (Q == 2 || Q == 4){
            m = m *-1;
        }
        double lineY = m * X;
        double lineX = m * Y;
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

    private PosPose2d l1Trough(){
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
