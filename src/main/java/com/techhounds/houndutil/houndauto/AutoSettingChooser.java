// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.techhounds.houndutil.houndauto;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.util.sendable.SendableRegistry;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

/**
 * The {@link SendableChooser} class is a useful tool for presenting a selection
 * of options to the
 * {@link SmartDashboard}.
 *
 * <p>
 * For instance, you may wish to be able to select between multiple autonomous
 * modes. You can do
 * this by putting every possible Command you want to run as an autonomous into
 * a {@link
 * SendableChooser} and then put it into the {@link SmartDashboard} to have a
 * list of options appear
 * on the laptop. Once autonomous starts, simply ask the {@link SendableChooser}
 * what the selected
 * value is.
 *
 */
public class AutoSettingChooser implements NTSendable, AutoCloseable {
    /** The key for the default value. */
    private static final String DEFAULT = "default";
    /** The key for the selected option. */
    private static final String SELECTED = "selected";
    /** The key for the active option. */
    private static final String ACTIVE = "active";
    /** The key for the option array. */
    private static final String OPTIONS = "options";
    /** The key for the instance number. */
    private static final String INSTANCE = ".instance";
    /** A map linking strings to the objects they represent. */
    private final Map<String, BaseAutoSettingValue> m_map = new LinkedHashMap<>();

    private String m_defaultChoice = "";
    private AutoSetting currentSetting;
    private int m_instance;
    private static final AtomicInteger s_instances = new AtomicInteger();

    /** Instantiates a {@link SendableChooser}. */
    public AutoSettingChooser() {
        m_instance = s_instances.getAndIncrement();
        SendableRegistry.add(this, "SendableChooser", m_instance);
    }

    @Override
    public void close() {
        SendableRegistry.remove(this);
        m_mutex.lock();
        try {
            for (StringPublisher pub : m_activePubs) {
                pub.close();
            }
        } finally {
            m_mutex.unlock();
        }
    }

    public AutoSetting getAutoSetting() {
        return currentSetting;
    }

    public String getName() {
        if (currentSetting != null)
            return currentSetting.getName();
        else
            return "";
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard}
     * on the desktop, the
     * object will appear as the given name.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void setAutoSetting(AutoSetting setting) {
        currentSetting = setting;
        m_map.clear();
        for (BaseAutoSettingValue value : setting.getPossibleValues()) {
            m_map.put(value.getName(), value);
        }
    }

    /**
     * Adds the given object to the list of options. On the {@link SmartDashboard}
     * on the desktop, the
     * object will appear as the given name.
     *
     * @param name   the name of the option
     * @param object the option
     */
    public void clearAutoSetting() {
        currentSetting = null;
        m_map.clear();
    }

    /**
     * Returns the selected option. If there is none selected, it will return the
     * default. If there is
     * none selected and no default, then it will return {@code null}.
     *
     * @return the option selected
     */
    public BaseAutoSettingValue getSelectedValue() {
        m_mutex.lock();
        try {
            if (m_selected != null) {
                return m_map.get(m_selected);
            } else {
                return m_map.get(m_defaultChoice);
            }
        } finally {
            m_mutex.unlock();
        }
    }

    public void updateSetting() {
        if (currentSetting != null)
            currentSetting.setValue(getSelectedValue());
    }

    private String m_selected;
    private final List<StringPublisher> m_activePubs = new ArrayList<>();
    private final ReentrantLock m_mutex = new ReentrantLock();

    @Override
    public void initSendable(NTSendableBuilder builder) {
        builder.setSmartDashboardType("String Chooser");
        IntegerPublisher instancePub = new IntegerTopic(builder.getTopic(INSTANCE)).publish();
        instancePub.set(m_instance);
        builder.addCloseable(instancePub);
        builder.addStringProperty(DEFAULT, () -> m_defaultChoice, null);
        builder.addStringArrayProperty(OPTIONS, () -> m_map.keySet().toArray(new String[0]), null);
        builder.addStringProperty(
                ACTIVE,
                () -> {
                    m_mutex.lock();
                    try {
                        if (m_selected != null) {
                            return m_selected;
                        } else {
                            return m_defaultChoice;
                        }
                    } finally {
                        m_mutex.unlock();
                    }
                },
                null);
        m_mutex.lock();
        try {
            m_activePubs.add(new StringTopic(builder.getTopic(ACTIVE)).publish());
        } finally {
            m_mutex.unlock();
        }
        builder.addStringProperty(
                SELECTED,
                null,
                val -> {
                    m_mutex.lock();
                    try {
                        m_selected = val;
                        for (StringPublisher pub : m_activePubs) {
                            pub.set(val);
                        }
                    } finally {
                        m_mutex.unlock();
                    }
                });
    }
}
