package org.firstinspires.ftc.teamcode.lib;


import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.apache.commons.io.FileUtils;
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.Random;
import java.util.Scanner;
import java.util.zip.ZipEntry;
import java.util.zip.ZipOutputStream;

import fi.iki.elonen.NanoHTTPD;

public class ServerShenanigans {
    private static final String MIME_ZIP = "application/zip";

    @WebHandlerRegistrar
    public static void attachWebServer(Context context, WebHandlerManager manager) {
        manager.register("/retrieve", session -> {
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
                zip.write(FileUtils.readFileToByteArray(file));
                zip.closeEntry();
            }

            // Create an InputStream of the output stream
            ByteArrayInputStream in = new ByteArrayInputStream(out.toByteArray());

            // Then, send the zip back to the client
            return NanoHTTPD.newChunkedResponse(
                    NanoHTTPD.Response.Status.OK,
                    MIME_ZIP,
                    in
            );
        });

        manager.register("/test_create", session -> {
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

        manager.register("/delete_all_data", session -> {
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
