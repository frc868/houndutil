package com.techhounds.houndutil.houndlog.interfaces;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import com.techhounds.houndutil.houndlog.enums.LogLevel;

@Retention(RetentionPolicy.RUNTIME)
@Target({ ElementType.FIELD, ElementType.METHOD })
public @interface Log {
    public String name();

    public String[] groups() default {};

    public LogLevel logLevel() default LogLevel.MAIN;

    public boolean array() default false;
}
