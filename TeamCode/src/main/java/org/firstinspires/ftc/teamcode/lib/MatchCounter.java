package org.firstinspires.ftc.teamcode.lib;

import android.content.Context;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;

/**
 * Maintains a file that contains a number identifying the match.
 */
@SuppressWarnings("ResultOfMethodCallIgnored")
public class MatchCounter {
    public static final String FILENAME = "match_counter.txt";
    public static final File FILE;
    private static int matchNumber;

    static {
        Context context = AppUtil.getInstance().getApplication();
        FILE = new File(context.getFilesDir(), FILENAME);
        matchNumber = readMatchNumber();
    }

    public static int newMatch() {
        writeMatchNumber(++matchNumber);
        RobotLog.ii("MatchCounter", "Starting match #" + matchNumber);
        return matchNumber;
    }

    public static int getMatchNumber() {
        return matchNumber;
    }

    public static void setMatchNumber(int x) {
        matchNumber = x;
        writeMatchNumber(matchNumber);
    }

    private static int readMatchNumber() {
        Context context = AppUtil.getInstance().getApplication();
        if (!FILE.exists()) return 0;
        try (FileInputStream fis = context.openFileInput(FILENAME)) {
            byte[] bytes = new byte[fis.available()];
            fis.read(bytes);
            return Integer.parseInt(new String(bytes, StandardCharsets.UTF_8));
        } catch (IOException e) {
            e.printStackTrace();
            return 0;
        }
    }

    private static void writeMatchNumber(int x) {
        Context context = AppUtil.getInstance().getApplication();
        FILE.delete();
        try (FileOutputStream fos = context.openFileOutput(FILENAME, Context.MODE_PRIVATE)) {
            fos.write(String.valueOf(x).getBytes(StandardCharsets.UTF_8));
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
