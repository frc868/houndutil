package com.techhounds.houndutil.houndlog.annotations;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * This annotation is used to mark a class as a class that should have its
 * values enumerated and searched for {@code @Log} annotations. These
 * annotations are searched recursively, so any number of nested classes can be
 * annotated with {@code @LoggedObject}.
 */
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface LoggedObject {
}
