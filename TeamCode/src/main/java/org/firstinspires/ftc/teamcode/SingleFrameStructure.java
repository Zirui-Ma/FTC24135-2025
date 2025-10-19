package org.firstinspires.ftc.teamcode;

public class SingleFrameStructure {

    private enum States{
        WAITING_TO_CAPTURE,
        CAPTURING,
        PROCESSING,
        OUTPUT
    }

    States currentState = States.WAITING_TO_CAPTURE;

    public void startCapturing() {
        currentState = States.CAPTURING;
    }

    public void loop() {
        switch(currentState) {
            case CAPTURING:
                break;
            case PROCESSING:
                break;
            case OUTPUT:
                break;
        }
    }
}
