package frc.FRC9485.Autonomous.autonomousChooser;

public interface AutoChooserIO {
    String getPathName();

    void addAutoNameCommand(String name, String pathName);
}
