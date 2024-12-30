package com.techhounds.houndutil.houndlog.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * An annotation used to mark fields that implement the Sendable interface to be
 * sent to a dashboard as a complex widget. By default, using {@code @Log} on an
 * object that implements Sendable will not use the Sendable interface, but
 * instead will attempt log the object using one of the defined profiles. Use
 * this annotation to force the object to be logged as a Sendable.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface SendableLog {
    /**
     * The name of the log value. If not provided, the name of the field is used.
     * 
     * @return the name of the log item
     */
    public String name() default "";

    /**
     * The values making up the subpath of the item. This is typically used to group
     * items together.
     * 
     * For example, specifying {@code control} in the groups parameter will result
     * in a path of {@code HoundLog/.../parent_name/control/value_name}
     * 
     * @return the groups of the log item
     */
    public String[] groups() default {};
}
