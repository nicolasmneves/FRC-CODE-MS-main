package frc.robot.Codigo.Visao;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.Codigo.Visao.CvCamera.ModoAlvo;
import frc.robot.Codigo.Visao.HSVdetection.VisionProfile;

/**
 * Comandos prontos de visao (command-based).
 * Nao exigem subsystem: so mudam configuracao ou esperam a camera.
 */
public final class ComandosVisao {

    /** Resultado mais velho que isso (s) nao conta como "vendo". */
    public static final double IDADE_MAXIMA = 0.3;

    private ComandosVisao() {
    }

    /** Troca o perfil. Ex: ComandosVisao.perfil(cam, VisionProfilesFactory.perfilGeral()) */
    public static Command perfil(CvCamera cam, VisionProfile perfil) {
        return Commands.runOnce(() -> cam.setPerfilManager(perfil)).ignoringDisable(true)
                .withName("Visao perfil " + perfil.getName());
    }

    /** So aceita alvo com esse nome ("" = qualquer). */
    public static Command seletor(CvCamera cam, String nome) {
        return Commands.runOnce(() -> cam.setSeletorAlvo(nome)).ignoringDisable(true)
                .withName("Visao seletor " + nome);
    }

    /** Troca o alvo prioritario para o proximo objeto (esquerda -> direita, volta ao primeiro). */
    public static Command proximoAlvo(CvCamera cam) {
        return Commands.runOnce(cam::proximoAlvo).ignoringDisable(true).withName("Visao proximo alvo");
    }

    public static Command modoAlvo(CvCamera cam, ModoAlvo modo) {
        return Commands.runOnce(() -> cam.setModoAlvo(modo)).ignoringDisable(true)
                .withName("Visao modo " + modo);
    }

    /** true se ha alvo recente. */
    public static boolean vendoAlvo(CvCamera cam) {
        return cam.temAlvo() && cam.getIdadeResultado() < IDADE_MAXIMA;
    }

    /** true se ha objeto com esse nome num resultado recente. */
    public static boolean vendoObjeto(CvCamera cam, String nome) {
        return cam.getIdadeResultado() < IDADE_MAXIMA && cam.existeObjeto(nome);
    }

    /** Espera ter alvo (ou estourar o tempo). */
    public static Command esperarAlvo(CvCamera cam, double timeoutSegundos) {
        return Commands.waitUntil(() -> vendoAlvo(cam)).withTimeout(timeoutSegundos)
                .withName("Visao esperar alvo");
    }

    /** Espera aparecer um objeto com esse nome (ou estourar o tempo). */
    public static Command esperarObjeto(CvCamera cam, String nome, double timeoutSegundos) {
        return Commands.waitUntil(() -> vendoObjeto(cam, nome)).withTimeout(timeoutSegundos)
                .withName("Visao esperar " + nome);
    }
}
