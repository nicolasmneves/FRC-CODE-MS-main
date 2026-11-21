package frc.robot.subsystems.Odometry.vision.hsv;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Perfil = lista de objetos procurados juntos + filtros.
 * Padrao: filtros desligados (mais leve).
 */
public class VisionProfile {

    private final String name;
    private final List<VisionObject> objects = new ArrayList<>();
    private final List<VisionObject> objectsLeitura = Collections.unmodifiableList(objects);

    private boolean blurEnabled = false;
    private int blurKernel = 0;
    private boolean morphEnabled = false;
    private int morphKernel = 0;
    private double brightness = 0;
    private double gamma = 1.0;

    public VisionProfile(String name) {
        this.name = name;
    }

    public VisionProfile addObject(VisionObject obj) {
        if (obj != null)
            objects.add(obj);
        return this;
    }

    public List<VisionObject> getObjects() {
        return objectsLeitura;
    }

    public String getName() {
        return name;
    }

    public int getBlurKernel() {
        return blurKernel;
    }

    public int getMorphKernel() {
        return morphKernel;
    }

    public double getBrightness() {
        return brightness;
    }

    public double getGamma() {
        return gamma;
    }

    /** Brilho 0 e gamma 1: a camera pula a correcao. */
    public boolean isCorrecaoNeutra() {
        return brightness == 0 && gamma == 1.0;
    }

    public boolean blurAtivo() {
        return blurEnabled && blurKernel >= 3 && (blurKernel % 2) == 1;
    }

    public boolean morphAtivo() {
        return morphEnabled && morphKernel >= 2;
    }

    /** medianBlur na mascara (tira pontinhos). Kernel impar: 3, 5, 7. */
    public void setBlur(boolean enabled, int kernel) {
        if (kernel > 0 && kernel % 2 == 0)
            kernel++;
        this.blurEnabled = enabled;
        this.blurKernel = kernel;
    }

    /** Morfologia abre+fecha na mascara (tira ruido e tapa buracos). Kernel 3-4 tipico. */
    public void setMorph(boolean enabled, int kernel) {
        this.morphEnabled = enabled;
        this.morphKernel = kernel;
    }

    /** Soma no brilho antes do HSV (-100..100). 0 = desligado. */
    public void setBrightness(double brightness) {
        this.brightness = Math.max(-255, Math.min(255, brightness));
    }

    /** Gamma antes do HSV (<1 clareia, >1 escurece). 1 = desligado. */
    public void setGamma(double gamma) {
        this.gamma = (gamma <= 0) ? 1.0 : gamma;
    }
}
