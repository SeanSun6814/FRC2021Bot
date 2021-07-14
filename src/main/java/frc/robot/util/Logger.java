package frc.robot.util;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.Timer;

public class Logger {

    private static PrintWriter writer = null;
    public static boolean debug = false;

    /**
     * Initializes the backend log file writer. Users do not need to manually call
     * this function. The logger function calls this whenever a log file writer is
     * needed.
     */
    private static void init() {
        if (writer != null)
            return;
        try {
            String time = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss").format(new Date());
            writer = new PrintWriter("/home/lvuser/Log " + time + ".csv");
            // writer = new PrintWriter("./Log" + time + ".csv");
            writer.println("Timestamp, Title, Message");
            System.out.println("Logger backend init.");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static void logRaw(Object message) {
        init();
        writer.print(message + "\n");
        if (debug)
            System.out.println(message);
    }

    @Deprecated
    public static void log(String title, Object msg) {
        logRaw(Timer.getFPGATimestamp() + "," + title + "," + msg);
    }

    /**
     * This actually makes sure all the logged messages gets saved to disk. This is
     * a expensive function, so call it in disabledInit().
     * 
     * The main robot program should be responsible for calling this, not each
     * individual subsystem.
     */
    public static void flush() {
        init();
        writer.flush();
        System.out.println("Log flushed");
    }

    private final String title;

    /**
     * Constructs a logger helper object for your subsystem. Everyone shares the
     * same log file and static log file writer, but each subsystem gets their
     * helper object so they don't have to input their own subsystem name every
     * time.
     * 
     * The static backend logger stuff will be automatically initialized when the
     * first subsystem regiesters or a static write method is called.
     * 
     * @param title
     */
    public Logger(String title) {
        init();
        this.title = title;
    }

    /**
     * Log a message under your subsystem name.
     * 
     * @param msg The message to log
     */
    public void log(Object msg) {
        logRaw(Timer.getFPGATimestamp() + "," + title + "," + msg);
    }

    // public static void main(String[] args) {
    // Logger l = new Logger("My Subsystem");
    // Logger ll = new Logger("My Second Subsystem");
    // Logger.debug = false;

    // long startTime = System.currentTimeMillis();

    // for (int i = 0; i < 1000; i++) {
    // l.log("message " + i);
    // ll.log("message " + i);
    // }
    // System.out.println((System.currentTimeMillis() - startTime) / 1000.0);
    // Logger.flush();
    // System.out.println((System.currentTimeMillis() - startTime) / 1000.0);
    // }

    // public static class Timer {
    // public static double time = 0;

    // public static double getFPGATimestamp() {
    // return System.currentTimeMillis();
    // }
    // }
}