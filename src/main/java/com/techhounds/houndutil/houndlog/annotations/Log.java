package com.techhounds.houndutil.houndlog.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

import com.techhounds.houndutil.houndlog.LogType;

/**
 * This is the main annotation used in HoundLog's annotation-based system.
 * Used to log the state of any field of method of an object that is registered
 * with HoundLog (typically using {@link LoggedObject}).
 * {@link LogAnnotationHandler} creates a new LogItem for each field or method
 * annotated with {@code @Log}.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target({ ElementType.FIELD, ElementType.METHOD })
public @interface Log {
    /**
     * The name of the log value. If not provided, the name of the field or method
     * is used.
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

    /**
     * The log type of the value. This will determine where the log value is
     * sent/stored. By default, this is {@link LogType#NT}.
     * 
     * @return the type of the log item.
     */
    public LogType logType() default LogType.NT;
}
