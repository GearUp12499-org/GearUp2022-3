package org.firstinspires.ftc.teamcode.lib;

import org.jetbrains.annotations.Contract;

public class NullTools {
    @Contract(value = "!null, _ -> param1; null, _ -> param2", pure = true)
    public static <T> T withDefault(T value, T defaultValue) {
        return value != null ? value : defaultValue;
    }
}
