package frc.robot.lib;

@FunctionalInterface
public interface AbsoluteEncoderFactory<Configuration> {
    AbsoluteEncoder create(Configuration configuration);
}