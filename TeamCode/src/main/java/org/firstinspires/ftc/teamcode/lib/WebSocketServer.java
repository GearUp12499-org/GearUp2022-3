package org.firstinspires.ftc.teamcode.lib;

import fi.iki.elonen.NanoWSD;

public class WebSocketServer extends NanoWSD {
    private static final int PORT = 8000;

    WebSocketServer() {
        super(PORT);
    }

    @Override
    protected WebSocket openWebSocket(IHTTPSession handshake) {
        return new WebSocketHandler(handshake);
    }
}
