package frc.robot.subsystems.Odometry.vision.hsv;

import java.io.File;
import java.util.Collections;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.opencv.core.Scalar;

import edu.wpi.first.wpilibj.Filesystem;

/**
 * Cadastro de OBJETOS (cores) e PERFIS da visao.
 *
 * COMO FUNCIONA
 *   Objeto = uma cor que a camera procura (nome, cor do contorno, area, faixas HSV).
 *   Perfil = lista de objetos procurados ao mesmo tempo + filtros.
 *   So um perfil fica ativo por vez: camera.setPerfilManager(perfil).
 *
 * PARA ADICIONAR UMA COR NOVA
 *   1. Crie um metodo criar_xxx() copiando criar_exemplo() abaixo.
 *   2. Coloque o objeto num perfil:  p.addObject(criar_xxx());
 *   3. (Perfil novo?) Registre em catalogo() para ele aparecer no Elastic.
 *
 * Os perfis sao montados uma vez so (cache); os JSONs nao sao relidos.
 */
public class VisionProfilesFactory {

    /** Pasta dos JSONs: src/main/deploy/Visao no projeto = /home/lvuser/deploy/Visao no robo. */
    private static final String HSV_DIR = Filesystem.getDeployDirectory().getAbsolutePath() + File.separator + "Visao"
            + File.separator;

    // =========================================================================
    // CATALOGO -- perfis que aparecem na lista "Visao/Config/Perfil" do Elastic
    // =========================================================================

    /** Nome mostrado no Elastic -> metodo que cria o perfil. Use o mesmo nome do perfil. */
    public static Map<String, Supplier<VisionProfile>> catalogo() {
        Map<String, Supplier<VisionProfile>> m = new LinkedHashMap<>();
        m.put("Desativado", VisionProfilesFactory::perfil_Desativado);
        m.put("Geral", VisionProfilesFactory::perfilGeral);
        // m.put("MeuPerfil", VisionProfilesFactory::perfil_MeuPerfil);
        return Collections.unmodifiableMap(m);
    }

    // =========================================================================
    // PERFIS
    // =========================================================================

    /** Nenhum objeto: a camera so transmite, sem processar cor (CPU minima). */
    public static VisionProfile perfil_Desativado() {
        return cache("Desativado", () -> new VisionProfile("Desativado"));
    }

    /** Perfil de uso geral: adicione aqui os objetos que quer procurar. */
    public static VisionProfile perfilGeral() {
        return cache("Geral", () -> {
            VisionProfile p = new VisionProfile("Geral");

            // objetos procurados (pode adicionar varios)
            // p.addObject(criar_cone_amarelo());
            // p.addObject(criar_cubo_marrom());;
            // p.addObject(criar_outraCor());

            // filtros na mascara (deixe desligado se nao precisar: mais leve)
            // p.setBlur(true, 5);     // tira pontinhos. Kernel impar: 3, 5, 7
            // p.setMorph(true, 4);  
              // tira ruido e tapa buracos. 3-4 tipico

            // correcao de imagem antes do HSV (0 e 1 = desligado)
            // p.setBrightness(0);  // -100..100
            // p.setGamma(1.0);     // <1 clareia, >1 escurece

            return p;
        });
    }

    // =========================================================================
    // OBJETOS
    // =========================================================================

    /**
     * Objeto de exemplo.
     *
     * new VisionObject(nome, corContorno, areaMin, areaMax)
     *   nome        -> aparece no Elastic e em camera.getObjetoDetectado()
     *   corContorno -> cor do contorno no stream, em BGR: new Scalar(Azul, Verde, Vermelho)
     *   areaMin     -> menor area aceita (px em 320x240). Filtra ruido.
     *   areaMax     -> maior area aceita (px em 320x240).
     *
     * Faixas HSV (escala OpenCV: H 0..179, S 0..255, V 0..255).
     * Use UMA das formas abaixo, ou as duas juntas (as faixas se somam):
     *
     *   A) Arquivo JSON do app de calibracao, em src/main/deploy/Visao/:
     *        .addHsvFromJson(HSV_DIR + "arquivo.json")
     *
     *   B) Manual: hmin, smin, vmin, hmax, smax, vmax
     *        .addHsvRange(100, 150, 50, 125, 255, 255)
     *      Varias de uma vez:
     *        .addHsvRanges(new double[] {0, 120, 70, 10, 255, 255},
     *                      new double[] {170, 120, 70, 179, 255, 255})
     *      (vermelho precisa de duas faixas: perto de H=0 e perto de H=179)
     */
    private static VisionObject criar_exemplo() {
        return new VisionObject("azul", new Scalar(255, 0, 0), 30, 10000)
                // A) arquivo do deploy
                .addHsvFromJson(HSV_DIR + "blocoazul.json");

                // B) ou manual (troque a linha acima por esta, ou use as duas)
                // .addHsvRange(100, 150, 50, 125, 255, 255);
    }
    private static VisionObject criar_cone_amarelo() {
        return new VisionObject("cone", new Scalar(60, 180, 216), 50, 20000)
             
                .addHsvFromJson(HSV_DIR + "coneamarelo.json");

              
    }
    private static VisionObject criar_cubo_marrom() {
        return new VisionObject("cubo", new Scalar(134, 177, 199), 50, 20000)
             
                .addHsvFromJson(HSV_DIR + "cubomarrom.json");

              
    }

    // =========================================================================
    // CACHE (nao precisa mexer)
    // =========================================================================

    private static final Map<String, VisionProfile> CACHE = new HashMap<>();

    private static synchronized VisionProfile cache(String chave, Supplier<VisionProfile> construtor) {
        return CACHE.computeIfAbsent(chave, k -> construtor.get());
    }

    /** Esquece os perfis montados (a proxima chamada rele os JSONs). */
    public static synchronized void limparCache() {
        CACHE.clear();
    }
}
