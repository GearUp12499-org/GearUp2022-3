package org.firstinspires.ftc.teamcode.lib;


import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.nio.charset.Charset;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Scanner;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import fi.iki.elonen.NanoHTTPD;

public class WebServerVer1 {
    private static final String MIME_ZIP = "application/zip";
    public static final Logger LOGGER = LoggerFactory.getLogger("File Server");

    public static String readFile(File file, Charset encoding) throws FileNotFoundException {
        Scanner scanner = new Scanner(file, encoding.name()).useDelimiter("\\A");
        String result = scanner.hasNext() ? scanner.next() : "";
        scanner.close();
        return result;
    }

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        manager.register("/files", session -> {
            File[] files_ = context.getFilesDir()
                    .listFiles((dir, name) -> name.startsWith("export_"));
            List<File> files = files_ == null ? Collections.emptyList() : Arrays.asList(files_);

            StringBuilder sb = new StringBuilder();
            sb
                    .append("<!DOCTYPE html>")
                    .append("<html>")
                    .append("<head>")
                    .append("<title>Files</title>")
                    .append("</head>")
                    .append("<body>")
                    .append("<h1>Files</h1>")
                    .append("<ul>");
            for (File file : files) {
                sb.append("<li>").append(file.getName()).append("</li>");
            }
            sb.append("</ul>")
                    .append("<br><a href=\"/files/retrieve\">Export</a>")
                    .append("<br><a href=\"/files/delete_all\">Delete All</a>")
                    .append("<br><br><a href=\"/files/test_create\">Create Dummy File</a>")
                    .append("</body>")
                    .append("</html>");
            return NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.OK,
                    NanoHTTPD.MIME_HTML, sb.toString()
            );
        });

        manager.register("/files/retrieve", session -> {
            // This is where the bulk of the code is.
            // <schema: NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws IOException, NanoHTTPD.ResponseException;>

            if (session.getMethod() != NanoHTTPD.Method.GET) {
                return NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED,
                        NanoHTTPD.MIME_PLAINTEXT, "GET only, sorry!"
                );
            }

            // Find all the files in internal storage that start with "export_"
            File[] files = context.getFilesDir()
                    .listFiles((dir, name) -> name.startsWith("export_"));

            if (files == null || files.length == 0) {
                return NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "No files found!"
                );
            }

            // Then use java.util.Zip to zip them up, and send it back
            // Hopefully have an InputStream or something at the end

            // piping
            final ByteArrayOutputStream out = new ByteArrayOutputStream();

            // zip with java.util.zip
            ZipOutputStream zip = new ZipOutputStream(out);
            for (File file : files) {
                zip.putNextEntry(new ZipEntry(file.getName()));
                String fd = readFile(file, StandardCharsets.UTF_8);
                LOGGER.info("fdata read " + fd.length());
                zip.write(fd.getBytes(StandardCharsets.UTF_8));

                zip.closeEntry();
            }
            zip.close();

            // Create an InputStream of the output stream
            ByteArrayInputStream in = new ByteArrayInputStream(out.toByteArray());

            // Then, send the zip back to the client
            return NanoHTTPD.newChunkedResponse(
                    NanoHTTPD.Response.Status.OK,
                    MIME_ZIP,
                    in
            );
        });

        manager.register("/files/test_create", session -> {
            Recorder testing = new Recorder();
            Random rng = new Random();
            for (int i = 0; i < 100; i++) {
                testing.put(rng.nextDouble()*15, rng.nextDouble()*100000);
            }
            testing.finalize(0);
            String name = "export_test_" + rng.nextInt(2048) + ".csv";
            testing.writeOnce(name);
            return NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.OK,
                    NanoHTTPD.MIME_PLAINTEXT,
                    "OK, new file at " + name
            );
        });

        manager.register("/files/delete_all", session -> {
            File[] files = context.getFilesDir()
                    .listFiles((dir, name) -> name.startsWith("export_"));
            if (files == null || files.length == 0) {
                return NanoHTTPD.newFixedLengthResponse(
                        NanoHTTPD.Response.Status.OK,
                        NanoHTTPD.MIME_PLAINTEXT, "No files to delete."
                );
            }
            final StringBuilder DATA = new StringBuilder("<!DOCTYPE html>" +
                    "<html>" +
                    "<head>" +
                    "<title>Delete All Data</title>" +
                    "</head>" +
                    "<body>" +
                    "<h1>Delete All Data</h1>" +
                    "Are you sure you want to delete <em>these " + files.length + " files</em>?<br>" +
                    "<ul>");

            for (File file : files) {
                DATA.append("<li>").append(file.getName()).append("</li>");
            }

            final String DATA2 = "</ul><form method=\"POST\"><input type=\"submit\" value=\"yes\"></form>" +
                    "</body>" +
                    "</html>";
            DATA.append(DATA2);
            switch (session.getMethod()) {
                case GET:
                    return NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK,
                            NanoHTTPD.MIME_HTML,
                            DATA.toString()
                    );
                case POST:
                    // Delete all files that start with "export_"
                    int count = 0;
                    for (File file : files) {
                        count += file.delete() ? 1 : 0;
                    }
                    return NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.OK,
                            NanoHTTPD.MIME_PLAINTEXT,
                            "Deleted " + count + " files"
                    );
                default:
                    return NanoHTTPD.newFixedLengthResponse(
                            NanoHTTPD.Response.Status.METHOD_NOT_ALLOWED,
                            NanoHTTPD.MIME_PLAINTEXT,
                            "Method not allowed"
                    );
            }
        });
    }
}
