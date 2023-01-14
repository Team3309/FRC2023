package friarLib2.hid;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LambdaTrigger extends Trigger {

    BooleanSupplier condition;

    public LambdaTrigger (BooleanSupplier condition) {
        this.condition = condition;
    }

    @Override
    public boolean get () {
        return condition.getAsBoolean();
    }
}
