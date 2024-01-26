package frc.lib.util;

public interface InverseInterpolable<T> {
    double inverseInterpolate(T upper, T query);
}
