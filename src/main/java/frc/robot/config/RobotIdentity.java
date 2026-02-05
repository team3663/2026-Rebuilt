package frc.robot.config;

public enum RobotIdentity {
    UNKNOWN("00-00-00-00-00-00"),
    TEST_BOT("00-80-2f-33-d0-1c"),
    C2025("00-80-2f-33-d0-3f"),
    C2026("00-00-00-00-00-00");

    private final String macAddress;

    RobotIdentity(String macAddress) {
        this.macAddress = macAddress;
    }

    public String getMacAddress() {
        return macAddress;
    }
}
