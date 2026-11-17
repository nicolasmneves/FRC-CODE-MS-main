package frc.robot.Codigo.Visao.HSVdetection;

import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.opencv.core.Scalar;

/**
 * Um objeto identificado por cor.
 *
 * Faixas HSV podem vir de:
 *   - arquivo do deploy:  .addHsvFromJson(caminho)
 *   - manual:             .addHsvRange(hmin, smin, vmin, hmax, smax, vmax)
 *   - varias manuais:     .addHsvRanges(new double[]{...}, new double[]{...})
 *   - os dois juntos (as faixas se somam).
 *
 * Areas em pixels medidas em 320x240 (a camera reescala em outras resolucoes).
 */
public class VisionObject {

    private final String name;
    private final Scalar drawColor;
    private final double minArea;
    private final double maxArea;

    private final List<HsvRange> ranges = new ArrayList<>();
    private final List<HsvRange> rangesLeitura = Collections.unmodifiableList(ranges);

    /**
     * @param name      nome do objeto (aparece nos resultados)
     * @param drawColor cor do contorno no stream, em BGR (Azul, Verde, Vermelho)
     * @param minArea   area minima do contorno (px, ref. 320x240)
     * @param maxArea   area maxima do contorno (px, ref. 320x240)
     */
    public VisionObject(String name, Scalar drawColor, double minArea, double maxArea) {
        this.name = name;
        this.drawColor = drawColor;
        this.minArea = minArea;
        this.maxArea = maxArea;
    }

    /** Adiciona uma faixa. Faixa ja coberta por outra e descartada (mesmo resultado, menos CPU). */
    public VisionObject addHsvRange(HsvRange range) {
        if (range == null)
            return this;
        for (HsvRange r : ranges) {
            if (r.contem(range))
                return this;
        }
        ranges.removeIf(range::contem);
        ranges.add(range);
        return this;
    }

    /** Faixa manual: hmin, smin, vmin, hmax, smax, vmax. */
    public VisionObject addHsvRange(double hmin, double smin, double vmin, double hmax, double smax, double vmax) {
        return addHsvRange(new HsvRange(hmin, smin, vmin, hmax, smax, vmax));
    }

    /** Varias faixas manuais. Cada array: {hmin, smin, vmin, hmax, smax, vmax}. */
    public VisionObject addHsvRanges(double[]... faixas) {
        if (faixas == null)
            return this;
        for (double[] f : faixas) {
            if (f == null || f.length != 6) {
                System.err.println("[Visao] '" + name + "': faixa manual precisa de 6 numeros -- ignorada");
                continue;
            }
            addHsvRange(f[0], f[1], f[2], f[3], f[4], f[5]);
        }
        return this;
    }

    // Leitor do JSON do app de calibracao: so le hmin..vmax de cada item de "samples".
    private static final Pattern P_SAMPLES = Pattern.compile("\"samples\"\\s*:\\s*\\[(.*?)\\]", Pattern.DOTALL);
    private static final Pattern P_OBJETO = Pattern.compile("\\{([^{}]*)\\}");
    private static final String[] CHAVES = { "hmin", "smin", "vmin", "hmax", "smax", "vmax" };
    private static final Pattern[] P_CHAVES = new Pattern[CHAVES.length];
    static {
        for (int i = 0; i < CHAVES.length; i++) {
            P_CHAVES[i] = Pattern.compile("\"" + CHAVES[i] + "\"\\s*:\\s*(-?[0-9]+(?:\\.[0-9]+)?)");
        }
    }

    /** Carrega as faixas de um JSON. Arquivo ausente/invalido: so avisa no console. */
    public VisionObject addHsvFromJson(String jsonPath) {
        String texto;
        try {
            texto = new String(Files.readAllBytes(Paths.get(jsonPath)), StandardCharsets.UTF_8);
        } catch (Exception e) {
            System.err.println("[Visao] '" + name + "': erro ao ler " + jsonPath + " -> " + e);
            return this;
        }

        Matcher mSamples = P_SAMPLES.matcher(texto);
        if (!mSamples.find()) {
            System.err.println("[Visao] '" + name + "': JSON sem \"samples\": " + jsonPath);
            return this;
        }

        int lidos = 0;
        int antes = ranges.size();
        Matcher mObj = P_OBJETO.matcher(mSamples.group(1));
        while (mObj.find()) {
            double[] v = new double[CHAVES.length];
            boolean ok = true;
            for (int i = 0; i < CHAVES.length && ok; i++) {
                Matcher m = P_CHAVES[i].matcher(mObj.group(1));
                if (m.find())
                    v[i] = Double.parseDouble(m.group(1));
                else
                    ok = false;
            }
            if (ok) {
                addHsvRange(v[0], v[1], v[2], v[3], v[4], v[5]);
                lidos++;
            }
        }

        System.out.println("[Visao] '" + name + "': " + lidos + " sample(s), " + (ranges.size() - antes)
                + " faixa(s) em uso -- " + jsonPath);
        return this;
    }

    public String getName() {
        return name;
    }

    public Scalar getDrawColor() {
        return drawColor;
    }

    public double getMinArea() {
        return minArea;
    }

    public double getMaxArea() {
        return maxArea;
    }

    public List<HsvRange> getRanges() {
        return rangesLeitura;
    }

    public boolean temFaixas() {
        return !ranges.isEmpty();
    }
}
