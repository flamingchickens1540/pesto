package org.team1540.robot2023.commands.music;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class MusicPlayer {
    private static final Orchestra orchestra = new Orchestra();

    public static ErrorCode addInstruments(TalonFX... instruments) {
        ErrorCode errorCode = ErrorCode.OK;
        for (TalonFX inst : instruments) {
            ErrorCode currentCode;
            if (errorCode == ErrorCode.OK && (currentCode = orchestra.addInstrument(inst)) != ErrorCode.OK)
                errorCode = currentCode;
        }
        return errorCode;
    }

    public static ErrorCode clearInstruments() {
        return orchestra.clearInstruments();
    }

    public static ErrorCode loadMusic(String songTitle) {
        return orchestra.loadMusic(songTitle + ".chrp");
    }

    public static ErrorCode play() {
        return orchestra.play();
    }

    public static ErrorCode stop() {
        return orchestra.stop();
    }

    public static ErrorCode pause() {
        return orchestra.pause();
    }

    public static boolean isPlaying() {
        return orchestra.isPlaying();
    }
}
