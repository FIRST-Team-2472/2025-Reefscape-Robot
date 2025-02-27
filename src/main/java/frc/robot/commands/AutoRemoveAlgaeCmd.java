package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSequences;
import frc.robot.Constants;
import frc.robot.subsystems.CoralDispenserSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoRemoveAlgaeCmd extends Command{
    CommandSequences commandSequences;
    ElevatorSubsystem elevatorSubsystem;
    SwerveSubsystem swerveSubsystem;
    //AlgaeArmSubsystem algaeArmSubsystem;
    int T, Q, P;     
    double X, Y; 
    double startElevatorHeight, endElevatorHeight;
    int algaeLocation; 
    Command removeAlgae;
    public AutoRemoveAlgaeCmd(CommandSequences commandSequences, ElevatorSubsystem elevatorSubsystem, SwerveSubsystem swerveSubsystem) {//algae arm subsystem when implemented
        this.commandSequences = commandSequences;
        this.elevatorSubsystem = elevatorSubsystem;
        this.swerveSubsystem = swerveSubsystem;
        //this.algaeArmSubsystem = algaeArmSubsystem;
        addRequirements(elevatorSubsystem, swerveSubsystem);//algae arm subsystem when implemented
    }
    @Override
    public void initialize() {      
        X = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
        Y = swerveSubsystem.getOdometer().getPoseMeters().getTranslation().getX();
        X-= 4.5;// reef x center
        Y-= 4.05;// reef y center
        Q = findQ();
        T = findT();
        P = findP();
        algaeLocation = findAlgaePos();
        if ((algaeLocation + 2) % 2 == 0){
            startElevatorHeight = Constants.ElevatorConstants.kElevatorL3AlgaeTop;
            endElevatorHeight = Constants.ElevatorConstants.kElevatorL3AlgaeBottom;
        } else {
            startElevatorHeight = Constants.ElevatorConstants.kElevatorL2AlgaeTop;
            endElevatorHeight = Constants.ElevatorConstants.kElevatorL2AlgaeBottom;
        }
        removeAlgae = commandSequences.removeAlgae(swerveSubsystem, elevatorSubsystem, algaeLocation, startElevatorHeight, endElevatorHeight);//algae arm subsystem when implemented
        removeAlgae.schedule();
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
                return 11;
            } else if (T == 2){
                return 12;
            } else {
                return 1;
            }
        } else if (Q == 3){
            if (T == 1){
                return 2;
            } else if (T == 2){
                return 3;
            } else {
                return 4;
            }
        } else if (Q == 4){
            if (T == 3){
                return 5;
            } else if (T == 2){
                return 6;
            } else {
                return 7;
            } 
        }else {
            if (T == 1){
                return 8;
            } else if (T == 2){
                return 9;
            } else {
                return 10;
            }
        }
    }

    private int findAlgaePos(){
        if (P == 1 || P == 2){
            return 0;
        } else if (P == 3 || P == 4){
            return 1;
        } else if (P == 5 || P == 6){
            return 2;
        } else if (P == 7 || P == 8){
            return 3;
        } else if (P == 9 || P == 10){
            return 4;
        } else {
            return 5;
        }
    }

    
}
