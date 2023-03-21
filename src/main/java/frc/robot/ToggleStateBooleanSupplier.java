package frc.robot;

import java.util.function.BooleanSupplier;

public class ToggleStateBooleanSupplier implements BooleanSupplier{

    private boolean state;

    public ToggleStateBooleanSupplier() {
        this(false);
    }

    public ToggleStateBooleanSupplier(boolean defaultState) {
        this.state = defaultState;
    }

    public void toggleState() {
        state = !state;
    }

    public boolean getState() {
        return state;
    }

    public void setState(boolean state) {
        this.state = state;
    }

    @Override
    public boolean getAsBoolean() {
        return state;
    }
    
}
