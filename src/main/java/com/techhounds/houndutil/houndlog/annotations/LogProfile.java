package com.techhounds.houndutil.houndlog.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to mark methods that generate {@link LogItem}s for a
 * specific type of object. This is used to create custom user-defined
 * profiles for logging data from desired objects.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ ElementType.METHOD })
public @interface LogProfile {
    /** The class that this profile represents. */
    public Class<?> value();
}
