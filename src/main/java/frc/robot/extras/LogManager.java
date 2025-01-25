package frc.robot.extras;

import java.io.PrintWriter;
import java.io.StringWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.LoggingConstants;


/*
Standard Data Logging using DataLogManager
The DataLogManager class (Java, C++, Python) provides a centralized data log that provides automatic data log file management. It automatically cleans up old files when disk space is low and renames the file based either on current date/time or (if available) competition match number. The data file will be saved to a USB flash drive in a folder called logs if one is attached, or to /home/lvuser/logs otherwise.

Note

USB flash drives need to be formatted as FAT32 to work with the roboRIO. NTFS or exFAT formatted drives will not work. Flash drives of 32GB or smaller are recommended, as Windows doesn’t format drives larger then 32GB with FAT32.

Log files are initially named FRC_TBD_{random}.wpilog until the DS connects. After the DS connects, the log file is renamed to FRC_yyyyMMdd_HHmmss.wpilog (where the date/time is UTC). If the FMS is connected and provides a match number, the log file is renamed to FRC_yyyyMMdd_HHmmss_{event}_{match}.wpilog.

On startup, all existing log files where a DS has not been connected will be deleted. If there is less than 50 MB of free space on the target storage, FRC_ log files are deleted (oldest to newest) until there is 50 MB free OR there are 10 files remaining.
 */
public class LogManager {


    // Logs at this message should be our most granular and only used for debugging purposes
    public static void debug(String message) {
        if (LoggingConstants.CURRENT_LOG_LEVEL.getSeverity() 
                <= System.Logger.Level.DEBUG.getSeverity()) {
            DataLogManager.log("DEBUG: " + message);
        }
    }

    // we should be more cautious about logging here and avoid too much data - avoid info logging for things tied to constant updates
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
