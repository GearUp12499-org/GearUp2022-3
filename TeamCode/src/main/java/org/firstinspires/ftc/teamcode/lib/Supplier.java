package org.firstinspires.ftc.teamcode.lib;

/**
 * Drop in replacement for java.util.function's Supplier.
 * Because the get(...) method isn't in my Android SDK version.
 *
 * @param <T> return type
 */
@FunctionalInterface
public interface Supplier<T> {

    /**
     * Gets a result.
     * @return a result
     */
    T get();
}
