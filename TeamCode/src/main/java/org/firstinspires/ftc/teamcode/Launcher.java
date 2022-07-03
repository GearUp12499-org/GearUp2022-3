package org.firstinspires.ftc.teamcode;

import android.content.Context;

import org.firstinspires.ftc.ftccommon.external.OnCreate;
import org.penguinencounter.server.SimpleHTTPServer;

public class Launcher {
    @OnCreate
    public static void bootServer(Context ignored) {
        SimpleHTTPServer.attachGet("/", (requestLine, headers, body) -> "Hello World!");
        Thread t = new Thread(SimpleHTTPServer::start);
        t.start();
    }
}
