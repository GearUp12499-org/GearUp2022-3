package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AutoLogTelemetry implements Telemetry {

    private final Telemetry wrapAround;
    private final VirtualTelemetryLog log;

    public AutoLogTelemetry(Telemetry wrapAround, VirtualTelemetryLog log) {
        this.wrapAround = wrapAround;
        this.log = log;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return wrapAround.addData(caption, format, args);
    }

    @Override
    public Item addData(String caption, Object value) {
        return wrapAround.addData(caption, value);
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        return wrapAround.addData(caption, valueProducer);
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        return wrapAround.addData(caption, format, valueProducer);
    }

    @Override
    public boolean removeItem(Item item) {
        return wrapAround.removeItem(item);
    }

    @Override
    public void clear() {
        wrapAround.clear();
    }

    @Override
    public void clearAll() {
        wrapAround.clearAll();
    }

    @Override
    public Object addAction(Runnable action) {
        return wrapAround.removeAction(action);
    }

    @Override
    public boolean removeAction(Object token) {
        return wrapAround.removeAction(token);
    }

    @Override
    public void speak(String text) {
        wrapAround.speak(text);
    }

    @Override
    public void speak(String text, String languageCode, String countryCode) {
        wrapAround.speak(text, languageCode, countryCode);
    }

    @Override
    public boolean update() {
        log.dump(this);
        return wrapAround.update();
    }

    @Override
    public Line addLine() {
        return wrapAround.addLine();
    }

    @Override
    public Line addLine(String lineCaption) {
        return wrapAround.addLine(lineCaption);
    }

    @Override
    public boolean removeLine(Line line) {
        return wrapAround.removeLine(line);
    }

    @Override
    public boolean isAutoClear() {
        return wrapAround.isAutoClear();
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        wrapAround.setAutoClear(autoClear);
    }

    @Override
    public int getMsTransmissionInterval() {
        return wrapAround.getMsTransmissionInterval();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        wrapAround.setMsTransmissionInterval(msTransmissionInterval);
    }

    @Override
    public String getItemSeparator() {
        return wrapAround.getItemSeparator();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        wrapAround.setItemSeparator(itemSeparator);
    }

    @Override
    public String getCaptionValueSeparator() {
        return wrapAround.getCaptionValueSeparator();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        wrapAround.setCaptionValueSeparator(captionValueSeparator);
    }

    @Override
    public void setDisplayFormat(DisplayFormat displayFormat) {
        wrapAround.setDisplayFormat(displayFormat);
    }

    @Override
    public Log log() {
        return wrapAround.log();
    }
}
