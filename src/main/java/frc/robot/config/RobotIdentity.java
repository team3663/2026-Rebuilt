package frc.robot.config;

public enum RobotIdentity {
    UNKNOWN("00-00-00-00-00-00"),
    TEST_BOT("00-80-2f-33-d0-1c"),
    C2025("00-80-2f-17-f9-1c"),
    C2026("00-80-2F-33-D0-1B");

    private final String macAddress;

    RobotIdentity(String macAddress) {
        this.macAddress = macAddress;
    }

    public String getMacAddress() {
        return macAddress;
    }
}
