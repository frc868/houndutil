package com.techhounds.houndutil.houndlog.interfaces;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.FIELD)
public @interface SendableLog {
    public String name() default "";

    public String[] groups() default {};

    public boolean array() default false;
}
