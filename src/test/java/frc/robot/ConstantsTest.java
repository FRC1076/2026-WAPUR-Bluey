package frc.robot;

import org.junit.jupiter.api.Test;

class ConstantsTest {
    @Test
    void allConstantsClassesInitialize() throws ClassNotFoundException {
        Class.forName(Constants.class.getName(), true, Constants.class.getClassLoader());
        for (Class<?> c : Constants.class.getDeclaredClasses()) {
            Class.forName(c.getName(), true, c.getClassLoader());
        }
    }
}