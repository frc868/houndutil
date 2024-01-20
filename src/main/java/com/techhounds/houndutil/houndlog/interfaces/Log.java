package com.techhounds.houndutil.houndlog.interfaces;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import com.techhounds.houndutil.houndlog.enums.LogType;

@Retention(RetentionPolicy.RUNTIME)
@Target({ ElementType.FIELD, ElementType.METHOD })
public @interface Log {
    public String name() default "";

    public String[] groups() default {};

    public LogType logLevel() default LogType.NT;

    public boolean array() default false;
}
