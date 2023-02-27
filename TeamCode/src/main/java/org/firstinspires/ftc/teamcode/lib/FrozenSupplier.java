package org.firstinspires.ftc.teamcode.lib;

public class FrozenSupplier<T> implements Supplier<T> {
    private final Supplier<T> around;
    private T result;
    private boolean resulted = false;

    public FrozenSupplier(Supplier<T> around) {
        this.around = around;
    }

    @Override
    public T get() {
        if (resulted) {
            return result;
        }
        resulted = true;
        result = around.get();
        return result;
    }
}
