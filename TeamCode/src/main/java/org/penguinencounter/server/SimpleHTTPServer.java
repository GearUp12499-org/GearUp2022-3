package org.penguinencounter.server;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class SimpleHTTPServer implements Runnable {
    static final int PORT = 8080;
    private Socket connection;
    public static boolean running = true;
    private static final Map<String, HTTPRequestConsumer> getHandlers = new HashMap<>();

    public static void stop() {
        running = false;
    }

    public SimpleHTTPServer(Socket connection) {
        this.connection = connection;
    }

    public static void start() {  // run in Thread, set "stop" flag to halt
        try {
            ServerSocket serverSocket = new ServerSocket(PORT);
            while (running) {
                SimpleHTTPServer serverConnection = new SimpleHTTPServer(serverSocket.accept());

                // dedicate a thread
                Thread thread = new Thread(serverConnection);
                thread.start();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public interface HTTPRequestConsumer {
        String accept(HTTPRequestLine requestLine, List<HTTPHeader> headers, List<String> body);
    }

    public static void attachGet(String path, HTTPRequestConsumer target) {
        getHandlers.put(path, target);
    }

    public enum HTTPRequestPosition {
        REQUEST,
        HEADER,
        BODY
    }

    public enum HTTPRequestMethod {
        GET("GET"),
        POST("POST"),
        PUT("PUT"),
        DELETE("DELETE"),
        HEAD("HEAD"),
        OPTIONS("OPTIONS"),
        CONNECT("CONNECT"),
        TRACE("TRACE"),
        PATCH("PATCH"),
        UNKNOWN("UNKNOWN");

        String name;
        HTTPRequestMethod(String method) {
            this.name = method;
        }
        public static HTTPRequestMethod getFromString(String method) {
            for (HTTPRequestMethod m : HTTPRequestMethod.values()) {
                if (m.name.equals(method)) {
                    return m;
                }
            }
            return UNKNOWN;
        }
    }

    public enum ResponseCodes {
        OK("200 OK"),
        NOT_FOUND("404 Not Found"),
        INTERNAL_SERVER_ERROR("500 Internal Server Error"),
        NOT_IMPLEMENTED("501 Not Implemented");

        String official;
        ResponseCodes(String official) {
            this.official = official;
        }
    }

    public static class HTTPHeader {
        public String name;
        public String value;
        public HTTPHeader(String name, String value) {
            this.name = name;
            this.value = value;
        }
        public static HTTPHeader parse(String line) {
            String[] parts = line.split(": ");
            return new HTTPHeader(parts[0], parts[1]);
        }
    }

    public static class HTTPRequestLine {
        public HTTPRequestMethod method;
        public String path;
        public String HTTPVersion;
        public HTTPRequestLine(HTTPRequestMethod method, String path, String HTTPVersion) {
            this.method = method;
            this.path = path;
            this.HTTPVersion = HTTPVersion;
        }
        public static HTTPRequestLine parse(String line) {
            String[] parts = line.split(" ");
            return new HTTPRequestLine(HTTPRequestMethod.getFromString(parts[0]), parts[1], parts[2]);
        }
    }

    public void process(BufferedReader in, PrintWriter out, BufferedOutputStream dataOut) throws IOException {
        HTTPRequestLine request = null;
        List<HTTPHeader> headers = new ArrayList<>();
        List<String> body = new ArrayList<>();
        HTTPRequestPosition position = HTTPRequestPosition.REQUEST;
        String line;
        while ((line = in.readLine()) != null) {
            switch (position) {
                case REQUEST:
                    request = HTTPRequestLine.parse(line);  // only one line
                    position = HTTPRequestPosition.HEADER;
                    break;
                case HEADER:
                    if (line.isEmpty()) {
                        position = HTTPRequestPosition.BODY;
                    } else {
                        headers.add(HTTPHeader.parse(line));
                    }
                    break;
                case BODY:
                    body.add(line);
                    break;
            }
        }
        if (request == null) throw new IOException("No request line");

        // OK, here we go
        if (request.method == HTTPRequestMethod.GET || request.method == HTTPRequestMethod.HEAD) {
            // don't distinguish between GET and HEAD, just send the same response
            // write
            if (getHandlers.containsKey(request.path)) {
                out.println(request.HTTPVersion + " " + ResponseCodes.OK.official);
                out.println("Content-Type: text/html");
                out.println();
                String response = getHandlers.get(request.path).accept(request, headers, body);
                out.println(response);
            } else {
                out.println(request.HTTPVersion + " " + ResponseCodes.NOT_FOUND.official);
            }
        } else {
            out.println(request.HTTPVersion + " " + ResponseCodes.NOT_IMPLEMENTED.official);
        }
        out.flush();
    }

    @Override
    public void run() {
        // Read in the socket data I guess
        BufferedReader in = null;
        PrintWriter out = null;
        BufferedOutputStream dataOut = null;

        try {
            // read in the request
            in = new BufferedReader(new InputStreamReader(connection.getInputStream()));
            out = new PrintWriter(connection.getOutputStream());
            dataOut = new BufferedOutputStream(connection.getOutputStream());
            // Time to parse some stuff!
            process(in, out, dataOut);
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            try {
                if (in != null) {
                    in.close();
                }
                if (out != null) {
                    out.close();
                }
                if (dataOut != null) {
                    dataOut.close();
                }
                connection.close();
            } catch (Exception ignore) {}
        }
    }
}
