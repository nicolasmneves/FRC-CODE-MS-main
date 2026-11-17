package frc.robot.Codigo.Dashboard;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Codigo.Sensores.LidarLiteV3;
import frc.robot.Codigo.Sensores.SensorFestoSOEG;
import frc.robot.Codigo.Visao.CvCamera;
import frc.robot.Codigo.Visao.CvCamera.ModoAlvo;
import frc.robot.Codigo.Visao.HSVdetection.Deteccao;
import frc.robot.Codigo.Visao.HSVdetection.VisionProfile;
import frc.robot.Codigo.Visao.HSVdetection.VisionProfilesFactory;

/**
 * Tudo que vai para o Elastic (NetworkTables 4, tabela SmartDashboard).
 *
 * No Elastic aparece em SmartDashboard/Visao:
 *   Status/  -> perfil, FPS, tempo, erro...      (somente leitura)
 *   Alvo/    -> tem alvo, nome, erro X, mm...     (somente leitura)
 *   Scan/    -> total e lista de objetos          (somente leitura)
 *   Config/  -> perfil, modo, seletor, stream, desenhar, mascara, exposicao (editavel)
 * Camera: CameraPublisher/VisaoHSV (widget Camera Stream).
 *
 * SmartDashboard/Sensores:
 *   Lidar/   -> distancia mm, conectado, leituras, erros
 *   Festo/   -> detectando, tensao (V)
 *
 * init() uma vez (depois de configurar os sistemas), atualizar() a cada ciclo.
 */
public class Dashboard {

    private static final String V = "Visao/";

    // Config (editaveis no Elastic)
    private static final String K_PERFIL = V + "Config/Perfil";
    private static final String K_MODO = V + "Config/Modo Alvo";
    private static final String K_SELETOR = V + "Config/Seletor Alvo";
    private static final String K_STREAM = V + "Config/Stream Ligado";
    private static final String K_DESENHAR = V + "Config/Desenhar Deteccoes";
    private static final String K_MASCARA = V + "Config/Mascara";
    private static final String K_EXPOSICAO = V + "Config/Exposicao (-1 auto)";

    private final CvCamera camera;
    private final LidarLiteV3 lidar;
    private final SensorFestoSOEG sensorFesto;
    private boolean iniciado = false;
    private final Map<String, Supplier<VisionProfile>> perfis = VisionProfilesFactory.catalogo();
    private final SendableChooser<String> escolhaPerfil = new SendableChooser<>();
    private final SendableChooser<ModoAlvo> escolhaModo = new SendableChooser<>();

    // ultimo valor visto em cada controle (so aplica quando o usuario muda)
    private String ultimoPerfil;
    private ModoAlvo ultimoModo;
    private String ultimoSeletor;
    private boolean ultimoStream;
    private boolean ultimoDesenhar;
    private String ultimaMascara;
    private double ultimaExposicao;

    public Dashboard(CvCamera camera, LidarLiteV3 lidar, SensorFestoSOEG sensorFesto) {
        this.camera = camera;
        this.lidar = lidar;
        this.sensorFesto = sensorFesto;
    }

    /** Chamar uma vez, depois de configurar camera e sensores. */
    public void init() {
        criarControlesVisao();
        iniciado = true;
    }

    /** Chamar em robotPeriodic(). */
    public void atualizar() {
        if (!iniciado)
            return;
        lerControlesVisao();
        publicarVisao();
        publicarSensores();
    }

    // =========================================================================
    // SENSORES (robo -> Elastic)
    // =========================================================================

    private void publicarSensores() {
        SmartDashboard.putNumber("Sensores/Lidar/Distancia mm", Math.round(lidar.getDistanciaMm()));
        SmartDashboard.putBoolean("Sensores/Lidar/Conectado", lidar.isConectado());
        SmartDashboard.putBoolean("Sensores/Lidar/Leitura OK", lidar.temLeitura());
        SmartDashboard.putNumber("Sensores/Lidar/Leituras", lidar.getLeiturasOk());
        SmartDashboard.putNumber("Sensores/Lidar/Erros", lidar.getErros());

        SmartDashboard.putBoolean("Sensores/Festo/Detectando", sensorFesto.getSensor());
        SmartDashboard.putNumber("Sensores/Festo/Tensao V", sensorFesto.getTensao());
    }

    // =========================================================================
    // VISAO -- controles (Elastic -> robo)
    // =========================================================================

    private void criarControlesVisao() {
        // perfil: lista do catalogo, comecando no perfil atual da camera
        String atual = camera.getPerfilAtivo().getName();
        if (!perfis.containsKey(atual))
            atual = "Desativado";
        for (String nome : perfis.keySet()) {
            if (nome.equals(atual))
                escolhaPerfil.setDefaultOption(nome, nome);
            else
                escolhaPerfil.addOption(nome, nome);
        }
        SmartDashboard.putData(K_PERFIL, escolhaPerfil);
        ultimoPerfil = atual;

        ModoAlvo modoAtual = camera.getModoAlvo();
        for (ModoAlvo m : ModoAlvo.values()) {
            if (m == modoAtual)
                escolhaModo.setDefaultOption(m.name(), m);
            else
                escolhaModo.addOption(m.name(), m);
        }
        SmartDashboard.putData(K_MODO, escolhaModo);
        ultimoModo = modoAtual;

        ultimoSeletor = camera.getSeletorAlvo();
        ultimoStream = camera.isStreamHabilitado();
        ultimaMascara = camera.getStreamMascara();
        ultimaExposicao = camera.getExposicao();
        SmartDashboard.putString(K_SELETOR, ultimoSeletor);
        SmartDashboard.putBoolean(K_STREAM, ultimoStream);
        SmartDashboard.putString(K_MASCARA, ultimaMascara);
        ultimoDesenhar = camera.isDesenharDeteccoes();
        SmartDashboard.putBoolean(K_DESENHAR, ultimoDesenhar);
        SmartDashboard.putNumber(K_EXPOSICAO, ultimaExposicao);
    }

    /**
     * Mudou no Elastic -> aplica na camera.
     * Mudou no codigo -> atualiza o Elastic (campos de texto/numero/booleano).
     */
    private void lerControlesVisao() {
        String perfil = escolhaPerfil.getSelected();
        if (perfil != null && !perfil.equals(ultimoPerfil)) {
            Supplier<VisionProfile> s = perfis.get(perfil);
            if (s != null)
                camera.setPerfilManager(s.get());
            ultimoPerfil = perfil;
        }

        ModoAlvo modo = escolhaModo.getSelected();
        if (modo != null && modo != ultimoModo) {
            camera.setModoAlvo(modo);
            ultimoModo = modo;
        }

        String seletor = SmartDashboard.getString(K_SELETOR, ultimoSeletor);
        if (!seletor.equals(ultimoSeletor)) {
            camera.setSeletorAlvo(seletor);
            ultimoSeletor = seletor;
        } else if (!camera.getSeletorAlvo().equals(seletor)) {
            ultimoSeletor = camera.getSeletorAlvo();
            SmartDashboard.putString(K_SELETOR, ultimoSeletor);
        }

        boolean stream = SmartDashboard.getBoolean(K_STREAM, ultimoStream);
        if (stream != ultimoStream) {
            camera.setStreamHabilitado(stream);
            ultimoStream = stream;
        } else if (camera.isStreamHabilitado() != stream) {
            ultimoStream = camera.isStreamHabilitado();
            SmartDashboard.putBoolean(K_STREAM, ultimoStream);
        }

        boolean desenhar = SmartDashboard.getBoolean(K_DESENHAR, ultimoDesenhar);
        if (desenhar != ultimoDesenhar) {
            camera.setDesenharDeteccoes(desenhar);
            ultimoDesenhar = desenhar;
        } else if (camera.isDesenharDeteccoes() != desenhar) {
            ultimoDesenhar = camera.isDesenharDeteccoes();
            SmartDashboard.putBoolean(K_DESENHAR, ultimoDesenhar);
        }

        String mascara = SmartDashboard.getString(K_MASCARA, ultimaMascara);
        if (!mascara.equals(ultimaMascara)) {
            camera.setStreamMascara(mascara);
            ultimaMascara = mascara;
        } else if (!camera.getStreamMascara().equals(mascara)) {
            ultimaMascara = camera.getStreamMascara();
            SmartDashboard.putString(K_MASCARA, ultimaMascara);
        }

        double exposicao = SmartDashboard.getNumber(K_EXPOSICAO, ultimaExposicao);
        if (exposicao != ultimaExposicao) {
            if (exposicao < 0)
                camera.setExposicaoAuto();
            else
                camera.setExposicaoManual((int) Math.round(exposicao));
            ultimaExposicao = exposicao;
        } else if (camera.getExposicao() != (int) Math.round(exposicao)) {
            ultimaExposicao = camera.getExposicao();
            SmartDashboard.putNumber(K_EXPOSICAO, ultimaExposicao);
        }
    }

    // =========================================================================
    // VISAO -- leitura (robo -> Elastic)
    // =========================================================================

    private void publicarVisao() {
        CvCamera.Resultado r = camera.getResultado();

        SmartDashboard.putString(V + "Status/Perfil", camera.getPerfilAtivo().getName());
        SmartDashboard.putString(V + "Status/Modo Alvo", camera.getModoAlvo().name());
        SmartDashboard.putBoolean(V + "Status/Rodando", camera.isRodando() && !camera.isPausado());
        SmartDashboard.putNumber(V + "Status/FPS", arred(camera.getFpsProcessamento()));
        SmartDashboard.putNumber(V + "Status/Tempo ms", arred(camera.getTempoProcessamentoMs()));
        SmartDashboard.putNumber(V + "Status/Idade s", arred(camera.getIdadeResultado()));
        SmartDashboard.putNumber(V + "Status/Latencia ms", Math.round(r.latencia() * 1000));
        SmartDashboard.putString(V + "Status/Resolucao", r.larguraFrame + "x" + r.alturaFrame);
        SmartDashboard.putString(V + "Status/Modo Camera", camera.getModoCamera());
        SmartDashboard.putBoolean(V + "Status/Camera Conectada", camera.isCameraConectada());
        SmartDashboard.putString(V + "Status/Erro", camera.getUltimoErro());

        Deteccao a = r.alvo;
        SmartDashboard.putBoolean(V + "Alvo/Tem Alvo", a != null);
        SmartDashboard.putString(V + "Alvo/Nome", a == null ? "" : a.nome);
        SmartDashboard.putString(V + "Alvo/Selecionado", camera.getIndiceAlvo() + " de " + camera.getTotalCandidatos());
        SmartDashboard.putNumber(V + "Alvo/Erro X", arred(camera.getErroXNormalizado()));
        SmartDashboard.putNumber(V + "Alvo/Profundidade mm", arred(r.profundidadeMm));
        SmartDashboard.putNumber(V + "Alvo/Lateral mm", arred(r.lateralCentroMm));
        SmartDashboard.putNumber(V + "Alvo/Vertical mm", arred(r.verticalCentroMm));
        SmartDashboard.putNumber(V + "Alvo/X px", a == null ? 0 : a.xCentro);
        SmartDashboard.putNumber(V + "Alvo/Y px", a == null ? 0 : a.yCentro);
        SmartDashboard.putNumber(V + "Alvo/Area px", a == null ? 0 : Math.round(a.area));

        List<Deteccao> lista = r.deteccoes;
        String[] nomes = new String[lista.size()];
        for (int i = 0; i < nomes.length; i++)
            nomes[i] = lista.get(i).toString();
        SmartDashboard.putNumber(V + "Scan/Total", nomes.length);
        SmartDashboard.putStringArray(V + "Scan/Objetos", nomes);
    }

    private static double arred(double v) {
        return Math.round(v * 100) / 100.0;
    }
}
