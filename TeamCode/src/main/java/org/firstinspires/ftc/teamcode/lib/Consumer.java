package org.firstinspires.ftc.teamcode.lib;


/**
 * Drop in replacement for java.lang.function's Consumer.
 * Because the accept(...) method isn't in my Android SDK version.
 *
 * @param <A> input type
 */
@FunctionalInterface
public interface Consumer<A> {
    void accept(A in);
}
