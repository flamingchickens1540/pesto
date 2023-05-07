package org.team1540.robot2023.commands.music;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;

public class MusicPlayer {
    private static MusicPlayer instance;
    private final Orchestra orchestra;

    private MusicPlayer() {
        orchestra = new Orchestra();
    }

    public static MusicPlayer getInstance() {
        if (instance == null) instance = new MusicPlayer();
        return instance;
    }

    public ErrorCode addInstruments(TalonFX... instruments) {
        ErrorCode errorCode = ErrorCode.OK;
        for (TalonFX inst : instruments) {
            ErrorCode currentCode;
            if (errorCode == ErrorCode.OK && (currentCode = orchestra.addInstrument(inst)) != ErrorCode.OK)
                errorCode = currentCode;
        }
        return errorCode;
    }

    public ErrorCode clearInstruments() {
        return orchestra.clearInstruments();
    }

    public ErrorCode loadMusic(String songTitle) {
        return orchestra.loadMusic("music/" + songTitle + ".chrp");
    }

    public ErrorCode play() {
        return orchestra.play();
    }

    public ErrorCode stop() {
        return orchestra.stop();
    }

    public ErrorCode pause() {
        return orchestra.pause();
    }

    public boolean isPlaying() {
        return orchestra.isPlaying();
    }
}
