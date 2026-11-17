package frc.robot.Codigo.Visao.HSVdetection;

/**
 * Um objeto encontrado num frame (imutavel).
 *
 * Coordenadas de IMAGEM (x, y, largura, altura): origem no canto superior esquerdo.
 * Coordenadas CARTESIANAS (xBorda, yBorda, xCentro, yCentro): 0 = centro da
 * imagem, + direita, + cima. Tudo em pixels.
 */
public final class Deteccao {

    public final VisionObject objeto;
    public final String nome;

    public final int x, y, largura, altura;
    public final double area;

    /** Canto superior esquerdo (cartesiano). */
    public final int xBorda, yBorda;
    /** Centro do retangulo (cartesiano). */
    public final int xCentro, yCentro;

    public Deteccao(VisionObject objeto, int x, int y, int largura, int altura, double area, double centroXpx,
            double centroYpx) {
        this.objeto = objeto;
        this.nome = objeto.getName();
        this.x = x;
        this.y = y;
        this.largura = largura;
        this.altura = altura;
        this.area = area;
        this.xBorda = (int) Math.round(x - centroXpx);
        this.yBorda = (int) Math.round(centroYpx - y);
        this.xCentro = (int) Math.round(x + largura / 2.0 - centroXpx);
        this.yCentro = (int) Math.round(centroYpx - (y + altura / 2.0));
    }

    @Override
    public String toString() {
        return nome + " | X:" + xCentro + " Y:" + yCentro + " A:" + Math.round(area);
    }
}
