package org.firstinspires.ftc.teamcode.lib;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

public class RisingFallingEdges {
    private static final Map<StackTraceElement, Boolean> lastState = new HashMap<>();

    private static StackTraceElement getCallSource() {
        Thread thread = Thread.currentThread();
        // Get first stack frame that is *not* in this class
        for (StackTraceElement stackElement : thread.getStackTrace()) {
            if (stackElement.getClassName().equals(RisingFallingEdges.class.getName())) continue;
            return stackElement;
        }
        throw new RuntimeException("cannot find call context");
    }
    public static boolean isRisingEdge(boolean value) {
        StackTraceElement el = getCallSource();
        boolean result = false;
        if (lastState.containsKey(el)) {
            result = value && Boolean.FALSE.equals(lastState.get(el));
        }
        lastState.put(el, value);
        return result;
    }
    public static boolean isFallingEdge(boolean value) {
        StackTraceElement el = getCallSource();
        boolean result = false;
        if (lastState.containsKey(el)) {
            result = !value && Boolean.TRUE.equals(lastState.get(el));
        }
        lastState.put(el, value);
        return result;
    }
    public static boolean isAnyEdge(boolean value) {
        StackTraceElement el = getCallSource();
        boolean result = false;
        if (lastState.containsKey(el)) {
            result = !Objects.equals(lastState.get(el), value);
        }
        lastState.put(el, value);
        return result;
    }
}
