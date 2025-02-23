package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class TestReefLocatoinMapping {
    int T, Q, P;     
    double X, Y; 
    String reefLocation;       

    @Test
    public void testReefLocationMapping() {
        X = 3.5;
        Y = 5.05;
        X-= 4.5;
        Y-= 4.05;
        Q = findQ();
        T = findT();
        P = findP();
        assertEquals(12, P);
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
                reefLocation = "K";
                return 11;
            } else if (T == 2){
                reefLocation = "L";
                return 12;
            } else {
                reefLocation = "A";
                return 1;
            }
        } else if (Q == 3){
            if (T == 1){
                reefLocation = "B";
                return 2;
            } else if (T == 2){
                reefLocation = "c";
                return 3;
            } else {
                reefLocation = "D";
                return 4;
            }
        } else if (Q == 4){
            if (T == 3){
                reefLocation = "E";
                return 5;
            } else if (T == 2){
                reefLocation = "F";
                return 6;
            } else {
                reefLocation = "G";
                return 7;
            } 
        }else {
            if (T == 1){
                reefLocation = "H";
                return 8;
            } else if (T == 2){
                reefLocation = "I";
                return 9;
            } else {
                reefLocation = "J";
                return 10;
            }
        }
    }

}
