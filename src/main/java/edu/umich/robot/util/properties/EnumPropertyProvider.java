/*
 * Copyright (c) 2010 Dave Ray <daveray@gmail.com>
 *
 * Created on May 29, 2010
 */
package edu.umich.robot.util.properties;

import java.util.concurrent.atomic.AtomicReference;

/**
 * Property provider for an enumeration.
 * 
 * @author ray
 */
public class EnumPropertyProvider<T extends Enum<T>> implements
        PropertyProvider<T>
{
    public final PropertyKey<T> key;

    private final AtomicReference<T> value;

    public EnumPropertyProvider(PropertyKey<T> key)
    {
        this.key = key;
        this.value = new AtomicReference<T>(key.getDefaultValue());
    }

    public T get()
    {
        return value.get();
    }

    public T set(T newValue)
    {
        if (newValue == null)
        {
            throw new NullPointerException("newValue shoudl not be null");
        }
        T oldValue = get();
        value.set(newValue);
        return oldValue;
    }

    /*
     * (non-Javadoc)
     * 
     * @see java.lang.Object#toString()
     */
    @Override
    public String toString()
    {
        return value.toString();
    }

}
