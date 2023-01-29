package org.firstinspires.ftc.teamcode.lib;

@FunctionalInterface
public interface Function<I, O> {
    O apply(I input);
}
