package frc.robot.Codigo.Visao.HSVdetection;

import org.opencv.core.Scalar;

/**
 * Uma faixa de cor HSV (minimo..maximo).
 * Escala OpenCV: H 0..179 | S 0..255 | V 0..255.
 */
public class HsvRange {

    private final Scalar lower;
    private final Scalar upper;

    public HsvRange(Scalar lower, Scalar upper) {
        this.lower = lower;
        this.upper = upper;
    }

    public HsvRange(double hmin, double smin, double vmin, double hmax, double smax, double vmax) {
        this(new Scalar(hmin, smin, vmin), new Scalar(hmax, smax, vmax));
    }

    public Scalar getLower() {
        return lower;
    }

    public Scalar getUpper() {
        return upper;
    }

    /** true se esta faixa cobre a outra inteira (a outra fica redundante). */
    public boolean contem(HsvRange outra) {
        for (int i = 0; i < 3; i++) {
            if (lower.val[i] > outra.lower.val[i] || upper.val[i] < outra.upper.val[i])
                return false;
        }
        return true;
    }

    @Override
    public String toString() {
        return String.format("H[%.0f..%.0f] S[%.0f..%.0f] V[%.0f..%.0f]", lower.val[0], upper.val[0], lower.val[1],
                upper.val[1], lower.val[2], upper.val[2]);
    }
}
