package org.firstinspires.ftc.teamcode.lib.interfaces

abstract class ISubsystem {
    abstract fun periodic()
    abstract fun read()
    abstract fun write()
    abstract fun reset()
}