package frc.robot.extras;

import java.io.PrintWriter;
import java.io.StringWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.LoggingConstants;

public class LogManager {

    // Logs at this message should be our most granular and only used for debugging purposes
    public static void debug(String message) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.DEBUG.getSeverity()) {
            DataLogManager.log("DEBUG: " + message);
        }
    }

    public static void info(String message) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.INFO.getSeverity()) {
            DataLogManager.log("INFO: " + message);
        }
    }

    public static void warning(String message) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.WARNING.getSeverity()) {
            DataLogManager.log("WARNING: " + message);
        }
    }

    public static void error(String message) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.ERROR.getSeverity()) {
            DataLogManager.log("ERROR: " + message);
        }
    }

    // Overloaded error method that accepts a Throwable
    public static void error(String message, Throwable throwable) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.ERROR.getSeverity()) {
            DataLogManager.log("ERROR: " + message + "\n" + stackTraceToString(throwable));
        }
    }

    /**
     * Helper method to convert a stack trace to a String.
     */
    private static String stackTraceToString(Throwable throwable) {
        StringWriter sw = new StringWriter();
        PrintWriter pw = new PrintWriter(sw);
        throwable.printStackTrace(pw);
        return sw.toString();
    }
}
