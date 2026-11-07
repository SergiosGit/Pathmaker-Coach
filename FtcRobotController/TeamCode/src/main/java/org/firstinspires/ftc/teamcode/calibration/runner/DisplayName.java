package org.firstinspires.ftc.teamcode.calibration.runner;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Label shown on the web page for an enum constant in a choice input. */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface DisplayName {
    String value();
}
