package com.techhounds.houndutil.houndauto;

import java.util.Arrays;

public class AutoSetting {
    private String name;

    private BaseAutoSettingValue[] possibleValues;
    private BaseAutoSettingValue value = null;

    public AutoSetting(String name, BaseAutoSettingValue[] possibleValues) {
        this.name = name;
        this.possibleValues = possibleValues;
    }

    public String getName() {
        return name;
    }

    public void setValue(BaseAutoSettingValue value) {
        if (Arrays.asList(possibleValues).contains(value)) {
            this.value = value;
        }
    }

    public BaseAutoSettingValue[] getPossibleValues() {
        return this.possibleValues;
    }

    public BaseAutoSettingValue getValue() {
        return value;
    }

}
