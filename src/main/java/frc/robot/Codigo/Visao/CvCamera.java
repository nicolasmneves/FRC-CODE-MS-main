package frc.robot.Codigo.Visao;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.Codigo.Visao.HSVdetection.Deteccao;
import frc.robot.Codigo.Visao.HSVdetection.HsvRange;
import frc.robot.Codigo.Visao.HSVdetection.VisionObject;
import frc.robot.Codigo.Visao.HSVdetection.VisionProfile;
import frc.robot.Codigo.Visao.HSVdetection.VisionProfilesFactory;

/**
 * Camera USB + deteccao por cor (HSV), em thread propria.
 *
 * Uso:
 *   CvCamera camera = new CvCamera();
 *   camera.start();
 *   camera.setPerfilManager(VisionProfilesFactory.perfilGeral());
 *   if (camera.temAlvo()) { ... camera.getErroXNormalizado() ... }
 *
 * Stream processado publicado pelo CameraServer (Elastic: CameraPublisher).
 * Desenha um retangulo sobre cada objeto, na cor do objeto (pode desligar:
 * setDesenharDeteccoes(false) mostra so a imagem da camera).
 *
 * Coordenadas: (0,0) = centro da imagem, X+ direita, Y+ cima.
 * Esta classe nao publica nada na dashboard (isso e do Dashboard.java).
 */
public class CvCamera {

    /** Grade 3x3 para ROI (linha, coluna de 1 a 3). */
    public enum BlocoROI {
        SUPERIOR_ESQUERDO(1, 1), SUPERIOR_CENTRO(1, 2), SUPERIOR_DIREITO(1, 3),
        CENTRO_ESQUERDO(2, 1), CENTRO(2, 2), CENTRO_DIREITO(2, 3),
        INFERIOR_ESQUERDO(3, 1), INFERIOR_CENTRO(3, 2), INFERIOR_DIREITO(3, 3);

        public final int linha;
        public final int coluna;

        BlocoROI(int linha, int coluna) {
            this.linha = linha;
            this.coluna = coluna;
        }
    }

    /** Como escolher o alvo entre varios objetos. */
    public enum ModoAlvo {
        ESQUERDA, CENTRO, DIREITA, MAIOR
    }

    /** Resultado de um frame (imutavel: todos os valores sao do mesmo frame). */
    public static final class Resultado {
        public final List<Deteccao> deteccoes;
        /** null se nao ha alvo. */
        public final Deteccao alvo;
        /** mm; -1 sem alvo. */
        public final double profundidadeMm;
        public final double lateralBordaMm, verticalBordaMm;
        public final double lateralCentroMm, verticalCentroMm;
        public final int larguraFrame, alturaFrame;
        /** Timer.getFPGATimestamp() de quando o frame terminou de ser processado. */
        public final double timestamp;
        /** Timer.getFPGATimestamp() de quando a camera capturou o frame (para compensar atraso). */
        public final double timestampCaptura;

        Resultado(List<Deteccao> deteccoes, Deteccao alvo, double profundidadeMm, double lateralBordaMm,
                double verticalBordaMm, double lateralCentroMm, double verticalCentroMm, int larguraFrame,
                int alturaFrame, double timestamp, double timestampCaptura) {
            this.deteccoes = deteccoes;
            this.alvo = alvo;
            this.profundidadeMm = profundidadeMm;
            this.lateralBordaMm = lateralBordaMm;
            this.verticalBordaMm = verticalBordaMm;
            this.lateralCentroMm = lateralCentroMm;
            this.verticalCentroMm = verticalCentroMm;
            this.larguraFrame = larguraFrame;
            this.alturaFrame = alturaFrame;
            this.timestamp = timestamp;
            this.timestampCaptura = timestampCaptura;
        }

        public boolean temAlvo() {
            return alvo != null;
        }

        static Resultado vazio(int w, int h) {
            double agora = Timer.getFPGATimestamp();
            return new Resultado(Collections.<Deteccao>emptyList(), null, -1, -1, -1, -1, -1, w, h, agora, agora);
        }

        /** Atraso total do frame (captura ate resultado pronto), em segundos. */
        public double latencia() {
            return timestamp - timestampCaptura;
        }
    }

    // ===== configuracao (qualquer thread escreve, a thread da camera le) =====

    private final int dispositivoUsb;
    private final String nomeStream;

    private volatile VisionProfile perfil;
    /** Regioes {x0,y0,x1,y1} normalizadas 0..1. */
    private volatile double[][] regioesRoi = { { 0, 0, 1, 1 } };
    private volatile ModoAlvo modoAlvo = ModoAlvo.CENTRO;
    private volatile String seletorAlvo = "";

    // ===== alvo prioritario (o objeto escolhido entre varios na tela) =====
    /** Botao "proximo alvo" pediu troca (atendido no proximo frame). */
    private volatile boolean pedirProximoAlvo = false;
    /** Pedido para esquecer o alvo travado e voltar a escolher pelo modo. */
    private volatile boolean pedirSoltarAlvo = false;
    /** Posicao do alvo prioritario (1 = mais a esquerda) e total de candidatos, para o Elastic. */
    private volatile int indiceAlvo = 0;
    private volatile int totalCandidatos = 0;

    private volatile int capturaW = 320;
    private volatile int capturaH = 240;
    private volatile int fps = 15;
    /** -1 = automatico. */
    private volatile int exposicao = -1;
    /** -1 = automatico. */
    private volatile int balancoBranco = -1;
    /** -1 = nao mexe. */
    private volatile int brilhoCamera = -1;
    /** true = YUYV (sem JPEG: evita "Corrupt JPEG data"). false = MJPEG. */
    private volatile boolean formatoYuyv = true;
    private boolean modosListados = false;
    /** Instante de captura do frame em processamento (so a thread da camera usa). */
    private double tCaptura = 0;
    // status lido pela dashboard (escrito so pela thread da camera)
    private volatile String modoCamera = "";
    private volatile boolean cameraConectada = false;
    private double ultimoStatusCamera = 0;
    private volatile boolean configCameraMudou = true;
    private volatile int passoProcessamento = 1;
    /** true = processa a imagem reduzida pela metade (mesma proporcao, ~4x mais rapido). */
    private volatile boolean processarMetade = false;

    private volatile boolean streamHabilitado = true;
    /** 0 = mesma resolucao do processamento. */
    private volatile int streamW = 0;
    private volatile int streamH = 0;
    private volatile int espessuraContorno = 2;
    /** true = desenha o retangulo de cada objeto no stream. false = so a imagem. */
    private volatile boolean desenharDeteccoes = true;
    /** Nome do objeto cuja mascara vai pro stream (calibracao). null = imagem normal. */
    private volatile String streamMascara = null;

    // calibracao geometrica (valores do projeto base, medidos em 320 de largura)
    private volatile double focalPixels = 277.0;
    private volatile double kProfundidade = 13898.0;
    private volatile int larguraBaseCalibracao = 320;
    private volatile double centroOffsetXpx = 0.0;
    private volatile double centroOffsetYpx = 0.0;

    // ===== saida =====

    private volatile Resultado resultado = Resultado.vazio(320, 240);
    private volatile double fpsProcessamento = 0;
    private volatile double tempoProcessamentoMs = 0;
    private volatile String ultimoErro = "";
    private volatile String erroConfig = "";

    private volatile boolean rodando = false;
    private volatile boolean pausado = false;
    private Thread thread;
    private UsbCamera camera;
    private CvSink cvSink;
    private CvSource output;

    /** Camera USB 0, stream "VisaoHSV". */
    public CvCamera() {
        this(0, "VisaoHSV");
    }

    /**
     * @param dispositivoUsb indice da camera USB (0, 1...)
     * @param nomeStream     nome do stream no Elastic (CameraPublisher)
     */
    public CvCamera(int dispositivoUsb, String nomeStream) {
        this.dispositivoUsb = dispositivoUsb;
        this.nomeStream = nomeStream;
        this.perfil = VisionProfilesFactory.perfil_Desativado();
    }

    // =========================================================================
    // CONTROLE
    // =========================================================================

    /** Abre a camera e inicia a thread. Chamar uma vez. */
    public synchronized void start() {
        if (thread != null)
            return;
        if (camera == null) {
            // UsbCamera direto: sem stream da imagem crua, so o processado (mais leve)
            camera = new UsbCamera("CamHSV" + dispositivoUsb, dispositivoUsb);
            cvSink = CameraServer.getVideo(camera);
            // publica o stream ja (aparece no Elastic mesmo sem frame ainda)
            boolean tamStream = streamW > 0 && streamH > 0;
            output = CameraServer.putVideo(nomeStream, tamStream ? streamW : capturaW, tamStream ? streamH : capturaH);
        }
        configCameraMudou = true;
        rodando = true;
        pausado = false;
        thread = new Thread(this::loop, "VisaoHSV-" + dispositivoUsb);
        thread.setDaemon(true);
        thread.start();
    }

    public synchronized void stop() {
        rodando = false;
        if (thread != null) {
            try {
                thread.join(1000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
            thread = null;
        }
    }

    /** Para de processar e limpa o alvo. */
    public void pause() {
        pausado = true;
        resultado = Resultado.vazio(resultado.larguraFrame, resultado.alturaFrame);
    }

    public void resume() {
        pausado = false;
    }

    public boolean isPausado() {
        return pausado;
    }

    public boolean isRodando() {
        return rodando;
    }

    public String getNomeStream() {
        return nomeStream;
    }

    // =========================================================================
    // PERFIL
    // =========================================================================

    /** Troca o perfil (vale no proximo frame). null = Desativado. */
    public void setPerfilManager(VisionProfile perfil) {
        this.perfil = (perfil == null) ? VisionProfilesFactory.perfil_Desativado() : perfil;
    }

    public VisionProfile getPerfilAtivo() {
        return perfil;
    }

    // =========================================================================
    // ROI
    // =========================================================================

    /** Ativa um ou mais blocos da grade 3x3 (substitui os atuais). */
    public void setRoiBlocos(BlocoROI... blocos) {
        if (blocos == null || blocos.length == 0) {
            roiCompleta();
            return;
        }
        regioesRoi = blocosParaRegioes(blocos);
    }

    /** Soma blocos aos ja ativos. */
    public void adicionarRoiBlocos(BlocoROI... blocos) {
        if (blocos == null || blocos.length == 0)
            return;
        double[][] novas = blocosParaRegioes(blocos);
        double[][] atuais = regioesRoi;
        double[][] combinadas = new double[atuais.length + novas.length][];
        System.arraycopy(atuais, 0, combinadas, 0, atuais.length);
        System.arraycopy(novas, 0, combinadas, atuais.length, novas.length);
        regioesRoi = combinadas;
    }

    /** ROI livre, 0..1, origem no canto superior esquerdo. */
    public void setRoiNormalizada(double x0, double y0, double x1, double y1) {
        regioesRoi = new double[][] { { clamp01(Math.min(x0, x1)), clamp01(Math.min(y0, y1)),
                clamp01(Math.max(x0, x1)), clamp01(Math.max(y0, y1)) } };
    }

    public void roiCompleta() {
        regioesRoi = new double[][] { { 0, 0, 1, 1 } };
    }

    public void roiMetadeSuperior() {
        regioesRoi = new double[][] { { 0, 0, 1, 0.5 } };
    }

    public void roiMetadeInferior() {
        regioesRoi = new double[][] { { 0, 0.5, 1, 1 } };
    }

    public void roiCentroHorizontal() {
        regioesRoi = new double[][] { { 0, 0.25, 1, 0.75 } };
    }

    public void roiEsquerdaVertical() {
        regioesRoi = new double[][] { { 0, 0, 1.0 / 3, 1 } };
    }

    public void roiCentroVertical() {
        regioesRoi = new double[][] { { 1.0 / 3, 0, 2.0 / 3, 1 } };
    }

    public void roiDireitaVertical() {
        regioesRoi = new double[][] { { 2.0 / 3, 0, 1, 1 } };
    }

    /** Quantidade de regioes de ROI ativas. */
    public int getRoiQuantidade() {
        return regioesRoi.length;
    }

    /** true se a ROI e a imagem inteira. */
    public boolean isRoiCompleta() {
        double[][] r = regioesRoi;
        return r.length == 1 && r[0][0] <= 0 && r[0][1] <= 0 && r[0][2] >= 1 && r[0][3] >= 1;
    }

    private static double[][] blocosParaRegioes(BlocoROI[] blocos) {
        double[][] regioes = new double[blocos.length][];
        for (int i = 0; i < blocos.length; i++) {
            BlocoROI b = blocos[i];
            regioes[i] = new double[] { (b.coluna - 1) / 3.0, (b.linha - 1) / 3.0, b.coluna / 3.0, b.linha / 3.0 };
        }
        return regioes;
    }

    // =========================================================================
    // ALVO
    // =========================================================================

    public void setModoAlvo(ModoAlvo modo) {
        if (modo != null && modo != this.modoAlvo) {
            this.modoAlvo = modo;
            soltarAlvo(); // escolhe de novo pelo modo novo
        }
    }

    public ModoAlvo getModoAlvo() {
        return modoAlvo;
    }

    /** So aceita alvo com esse nome. "" = qualquer objeto. */
    public void setSeletorAlvo(String nome) {
        String novo = (nome == null) ? "" : nome.trim();
        if (!novo.equals(seletorAlvo))
            soltarAlvo();
        this.seletorAlvo = novo;
    }

    public String getSeletorAlvo() {
        return seletorAlvo;
    }

    /**
     * Troca o alvo prioritario para o proximo objeto na tela, da esquerda para a
     * direita; depois do ultimo volta ao primeiro. (Botao RB.)
     */
    public void proximoAlvo() {
        pedirProximoAlvo = true;
    }

    /** Esquece o alvo travado: o proximo frame escolhe de novo pelo modo (CENTRO...). */
    public void soltarAlvo() {
        pedirSoltarAlvo = true;
    }

    /** Posicao do alvo prioritario contando da esquerda (1, 2, ...). 0 = sem alvo. */
    public int getIndiceAlvo() {
        return indiceAlvo;
    }

    /** Quantos objetos podem ser alvo agora (respeitando o seletor por nome). */
    public int getTotalCandidatos() {
        return totalCandidatos;
    }

    // =========================================================================
    // CAMERA / QUALIDADE
    // =========================================================================

    /** Resolucao de captura e processamento. A camera precisa suportar o modo. */
    public void setQualidadeProcessamento(int largura, int altura) {
        if (largura > 0 && altura > 0) {
            this.capturaW = largura;
            this.capturaH = altura;
            this.configCameraMudou = true;
        }
    }

    public void setQualidade160() {
        setQualidadeProcessamento(160, 120);
    }

    public void setQualidade320() {
        setQualidadeProcessamento(320, 240);
    }

    /** Pesado no roboRIO. */
    public void setQualidade640() {
        setQualidadeProcessamento(640, 480);
    }

    public int getQualidadeProcessamentoW() {
        return capturaW;
    }

    public int getQualidadeProcessamentoH() {
        return capturaH;
    }

    /** FPS da camera. Menos FPS = menos CPU. */
    public void setFps(int fps) {
        if (fps > 0) {
            this.fps = fps;
            this.configCameraMudou = true;
        }
    }

    public int getFps() {
        return fps;
    }

    /** Processa 1 a cada n frames. Prefira setFps(). */
    public void setPassoProcessamento(int n) {
        this.passoProcessamento = Math.max(1, n);
    }

    /**
     * Processa a imagem reduzida pela metade (ex: captura 320x240, processa 160x120).
     * Mantem a proporcao (sem distorcer) e deixa a deteccao ~4x mais leve.
     * Areas, focal e profundidade se ajustam sozinhos. O stream sai na metade tambem.
     */
    public void setProcessarMetade(boolean metade) {
        this.processarMetade = metade;
    }

    /** Exposicao manual 0..100 (recomendado para cores estaveis). */
    public void setExposicaoManual(int valor) {
        this.exposicao = Math.max(0, Math.min(100, valor));
        this.configCameraMudou = true;
    }

    public void setExposicaoAuto() {
        this.exposicao = -1;
        this.configCameraMudou = true;
    }

    /** -1 = automatica. */
    public int getExposicao() {
        return exposicao;
    }

    /** Balanco de branco manual em Kelvin (ex: 4500). */
    public void setBalancoBrancoManual(int kelvin) {
        this.balancoBranco = Math.max(0, kelvin);
        this.configCameraMudou = true;
    }

    public void setBalancoBrancoAuto() {
        this.balancoBranco = -1;
        this.configCameraMudou = true;
    }

    /** Brilho no hardware da camera 0..100. */
    public void setBrilhoCamera(int valor) {
        this.brilhoCamera = Math.max(0, Math.min(100, valor));
        this.configCameraMudou = true;
    }

    /**
     * Formato pedido a camera. YUYV (padrao) nao usa JPEG: resolve camera que
     * manda JPEG corrompido. MJPEG gasta menos USB (use se a camera nao tiver YUYV).
     */
    public void setFormatoYUYV(boolean yuyv) {
        this.formatoYuyv = yuyv;
        this.configCameraMudou = true;
    }

    /** Modo real da camera, ex: "kYUYV 320x240 30fps" (atualizado pela thread da camera). */
    public String getModoCamera() {
        return modoCamera;
    }

    /** true se a camera esta entregando frames. */
    public boolean isCameraConectada() {
        return cameraConectada;
    }

    /** Le o modo real da camera. So a thread da camera chama (cscore pode lancar excecao). */
    private void atualizarStatusCamera() {
        try {
            VideoMode m = camera.getVideoMode();
            modoCamera = m.pixelFormat + " " + m.width + "x" + m.height + " " + m.fps + "fps";
        } catch (Exception e) {
            modoCamera = "desconectada";
        }
    }

    // =========================================================================
    // STREAM
    // =========================================================================

    /** Desligado = mais leve (deteccao continua). */
    public void setStreamHabilitado(boolean on) {
        this.streamHabilitado = on;
    }

    public boolean isStreamHabilitado() {
        return streamHabilitado;
    }

    /** Resolucao do stream. (0,0) = igual ao processamento. */
    public void setQualidadeStream(int largura, int altura) {
        this.streamW = Math.max(0, largura);
        this.streamH = Math.max(0, altura);
    }

    /** Espessura da linha do retangulo no stream (1..10 px). */
    public void setEspessuraContorno(int px) {
        this.espessuraContorno = Math.max(1, Math.min(10, px));
    }

    /**
     * Liga/desliga o retangulo sobre os objetos no stream.
     * Desligado: stream mostra so a camera (menos trabalho por frame).
     * A deteccao continua igual (posicoes e alvo seguem funcionando).
     */
    public void setDesenharDeteccoes(boolean on) {
        this.desenharDeteccoes = on;
    }

    public boolean isDesenharDeteccoes() {
        return desenharDeteccoes;
    }

    /** Stream mostra a mascara (preto/branco) deste objeto. null ou "" = normal. */
    public void setStreamMascara(String nomeObjeto) {
        this.streamMascara = (nomeObjeto == null || nomeObjeto.trim().isEmpty()) ? null : nomeObjeto.trim();
    }

    /** "" quando desligado. */
    public String getStreamMascara() {
        String m = streamMascara;
        return (m == null) ? "" : m;
    }

    // =========================================================================
    // CALIBRACAO (pixel -> mm)
    // =========================================================================

    /** Largura atual / largura de calibracao. */
    public double escalaResolucao() {
        int base = larguraBaseCalibracao;
        int w = resultado.larguraFrame;
        if (base <= 0 || w <= 0)
            return 1.0;
        return w / (double) base;
    }

    /** Largura em que focal, K e areas foram calibrados (padrao 320). */
    public void setLarguraBaseCalibracao(int larguraPx) {
        if (larguraPx > 0)
            this.larguraBaseCalibracao = larguraPx;
    }

    public int getLarguraBaseCalibracao() {
        return larguraBaseCalibracao;
    }

    /** Distancia focal em px (na largura base). */
    public void setFocalPixels(double focalPixels) {
        if (focalPixels > 0)
            this.focalPixels = focalPixels;
    }

    public double getFocalPixels() {
        return focalPixels;
    }

    /** profundidade(mm) = K / largura(px), na largura base. */
    public void setKProfundidade(double k) {
        if (k > 0)
            this.kProfundidade = k;
    }

    public double getKProfundidade() {
        return kProfundidade;
    }

    /** Ajuste do centro optico em px (+ direita / + baixo). */
    public void setCentroOffset(double offsetXpx, double offsetYpx) {
        this.centroOffsetXpx = offsetXpx;
        this.centroOffsetYpx = offsetYpx;
    }

    public double pixelsParaMm(double deltaPx, double profundidadeMm) {
        double f = focalPixels * escalaResolucao();
        if (f <= 0 || profundidadeMm <= 0)
            return 0;
        return deltaPx * profundidadeMm / f;
    }

    // =========================================================================
    // RESULTADOS -- ALVO
    // =========================================================================

    public Resultado getResultado() {
        return resultado;
    }

    public boolean temAlvo() {
        return resultado.alvo != null;
    }

    /** null se nao ha alvo. */
    public Deteccao getAlvo() {
        return resultado.alvo;
    }

    /** Nome do alvo ou "". */
    public String getObjetoDetectado() {
        Deteccao a = resultado.alvo;
        return (a == null) ? "" : a.nome;
    }

    public int getTotalContornos() {
        return resultado.deteccoes.size();
    }

    /** mm; -1 sem alvo. */
    public double getProfundidade() {
        return resultado.profundidadeMm;
    }

    /** mm pela borda esquerda; -1 sem alvo (use temAlvo()). */
    public double getPosicaoLateralBorda() {
        return resultado.lateralBordaMm;
    }

    /** mm pela borda superior; -1 sem alvo. */
    public double getPosicaoVerticalBorda() {
        return resultado.verticalBordaMm;
    }

    /** mm pelo centro; -1 sem alvo. */
    public double getPosicaoLateralCentro() {
        return resultado.lateralCentroMm;
    }

    /** mm pelo centro; -1 sem alvo. */
    public double getPosicaoVerticalCentro() {
        return resultado.verticalCentroMm;
    }

    /** px de imagem; -1 sem alvo. */
    public double getPosicaoLateralBordaPx() {
        Deteccao a = resultado.alvo;
        return (a == null) ? -1 : a.x;
    }

    public double getPosicaoVerticalBordaPx() {
        Deteccao a = resultado.alvo;
        return (a == null) ? -1 : a.y;
    }

    public double getPosicaoLateralCentroPx() {
        Deteccao a = resultado.alvo;
        return (a == null) ? -1 : a.x + a.largura / 2.0;
    }

    public double getPosicaoVerticalCentroPx() {
        Deteccao a = resultado.alvo;
        return (a == null) ? -1 : a.y + a.altura / 2.0;
    }

    /** -1 (esquerda) .. 0 (centro) .. +1 (direita). 0 sem alvo. */
    public double getErroXNormalizado() {
        Resultado r = resultado;
        if (r.alvo == null || r.larguraFrame <= 0)
            return 0;
        return r.alvo.xCentro / (r.larguraFrame / 2.0);
    }

    /** Segundos desde o ultimo frame processado. */
    public double getIdadeResultado() {
        return Timer.getFPGATimestamp() - resultado.timestamp;
    }

    public double getFpsProcessamento() {
        return fpsProcessamento;
    }

    public double getTempoProcessamentoMs() {
        return tempoProcessamentoMs;
    }

    /** "" se tudo certo. */
    public String getUltimoErro() {
        String e = ultimoErro;
        return e.isEmpty() ? erroConfig : e;
    }

    // =========================================================================
    // RESULTADOS -- TODOS OS OBJETOS
    // =========================================================================

    /** Todas as deteccoes do ultimo frame (somente leitura). */
    public List<Deteccao> getDeteccoes() {
        return resultado.deteccoes;
    }

    public List<Deteccao> getDeteccoes(String nome) {
        List<Deteccao> out = new ArrayList<>();
        for (Deteccao d : resultado.deteccoes) {
            if (d.nome.equalsIgnoreCase(nome))
                out.add(d);
        }
        return out;
    }

    public int contarObjetos(String nome) {
        int n = 0;
        for (Deteccao d : resultado.deteccoes) {
            if (d.nome.equalsIgnoreCase(nome))
                n++;
        }
        return n;
    }

    public boolean existeObjeto(String nome) {
        return contarObjetos(nome) > 0;
    }

    /** Nome do objeto mais proximo do ponto (cartesiano, pela borda) ou "". */
    public String buscarObjetoImagem(int alvoX, int alvoY, int toleranciaX, int toleranciaY) {
        return buscar(alvoX, alvoY, toleranciaX, toleranciaY, false);
    }

    /** Igual buscarObjetoImagem, pelo centro do objeto. */
    public String buscarObjetoImagemPorCentro(int alvoX, int alvoY, int toleranciaX, int toleranciaY) {
        return buscar(alvoX, alvoY, toleranciaX, toleranciaY, true);
    }

    private String buscar(int alvoX, int alvoY, int tolX, int tolY, boolean porCentro) {
        String melhorNome = "";
        long melhorDist = Long.MAX_VALUE;
        for (Deteccao d : resultado.deteccoes) {
            int dx = (porCentro ? d.xCentro : d.xBorda) - alvoX;
            int dy = (porCentro ? d.yCentro : d.yBorda) - alvoY;
            if (Math.abs(dx) <= tolX && Math.abs(dy) <= tolY) {
                long dist = (long) dx * dx + (long) dy * dy;
                if (dist < melhorDist) {
                    melhorDist = dist;
                    melhorNome = d.nome;
                }
            }
        }
        return melhorNome;
    }

    /** Nomes da esquerda para a direita; posicoes que sobram ficam null. */
    public void organizarScanPorXCrescente(String[] destino) {
        if (destino == null)
            return;
        List<Deteccao> copia = new ArrayList<>(resultado.deteccoes);
        copia.sort((a, b) -> Integer.compare(a.xBorda, b.xBorda));
        for (int i = 0; i < destino.length; i++)
            destino[i] = (i < copia.size()) ? copia.get(i).nome : null;
    }

    // =========================================================================
    // THREAD DA CAMERA (tudo abaixo roda so nela)
    // =========================================================================

    private Mat lut;
    private double lutBrilho = Double.NaN;
    private double lutGamma = Double.NaN;
    private Mat kernelMorph;
    private int kernelMorphTam = -1;
    private Mat roiMaskCache;
    private double[][] roiMaskRegioes;
    private int roiMaskW = -1, roiMaskH = -1;

    private static final Scalar BRANCO = new Scalar(255);
    private static final Scalar VERDE_FLUOR = new Scalar(20, 255, 57);   // BGR
    private static final Scalar PRETO = new Scalar(0, 0, 0);

    // rastreio do alvo prioritario (so a thread da camera usa)
    /** Distancia maxima (fracao da largura da imagem) para achar o mesmo objeto no frame seguinte. */
    private static final double RAIO_RASTREIO = 0.25;
    /** Frames seguidos sem achar o alvo antes de esquecer ele (~0,5 s a 20 fps). */
    private static final int FRAMES_PERDA = 10;
    private boolean rastreando = false;
    private int rastreioX, rastreioY;
    private int framesSemAlvo = 0;

    private void loop() {
        // buffers reutilizados (sem alocar por frame)
        lut = new Mat(1, 256, CvType.CV_8UC1);
        Mat frame = new Mat();
        Mat corrigido = new Mat();
        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat temp = new Mat();
        Mat hierarquia = new Mat();
        Mat mascaraDebug = new Mat();
        Mat streamFrame = new Mat();
        Mat frameMetade = new Mat();
        List<MatOfPoint> contornos = new ArrayList<>();

        long contador = 0;
        double fpsT0 = Timer.getFPGATimestamp();
        int fpsFrames = 0;

        try {
            while (rodando) {
                if (pausado) {
                    dormir(50);
                    continue;
                }

                if (configCameraMudou) {
                    configCameraMudou = false;
                    aplicarConfigCamera();
                }

                long tempoFrameUs = cvSink.grabFrame(frame, 0.5);
                if (tempoFrameUs == 0) {
                    ultimoErro = "grabFrame: " + cvSink.getError();
                    cameraConectada = false;
                    modoCamera = "desconectada";
                    dormir(20);
                    continue;
                }
                cameraConectada = true;
                // instante da captura (mesma base do Timer no roboRIO); se vier fora, usa agora
                double agoraCaptura = Timer.getFPGATimestamp();
                tCaptura = tempoFrameUs / 1e6;
                if (Math.abs(agoraCaptura - tCaptura) > 1.0)
                    tCaptura = agoraCaptura;

                double agoraStatus = Timer.getFPGATimestamp();
                if (agoraStatus - ultimoStatusCamera >= 1.0) {
                    atualizarStatusCamera();
                    ultimoStatusCamera = agoraStatus;
                }

                if (!modosListados) {
                    listarModos(); // uma vez: mostra no console o que a camera suporta
                    modosListados = true;
                }

                if ((contador++ % Math.max(1, passoProcessamento)) != 0)
                    continue;

                long t0 = System.nanoTime();
                boolean mandaStream = streamHabilitado;
                Mat entrada = frame;
                if (processarMetade) {
                    Imgproc.resize(frame, frameMetade, new Size(frame.cols() / 2, frame.rows() / 2), 0, 0,
                            Imgproc.INTER_AREA);
                    entrada = frameMetade;
                }
                Mat saida = entrada;
                try {
                    Mat m = processarFrame(entrada, corrigido, hsv, mask, temp, hierarquia, contornos, mascaraDebug,
                            mandaStream);
                    if (m != null)
                        saida = m;
                    ultimoErro = "";
                } catch (Throwable e) {
                    ultimoErro = e.toString(); // um frame ruim nao derruba a thread
                }

                double ms = (System.nanoTime() - t0) / 1e6;
                tempoProcessamentoMs = tempoProcessamentoMs * 0.9 + ms * 0.1;
                fpsFrames++;
                double agora = Timer.getFPGATimestamp();
                if (agora - fpsT0 >= 1.0) {
                    fpsProcessamento = fpsFrames / (agora - fpsT0);
                    fpsFrames = 0;
                    fpsT0 = agora;
                }

                if (mandaStream)
                    enviarStream(saida, streamFrame);
            }
        } finally {
            frame.release();
            corrigido.release();
            hsv.release();
            mask.release();
            temp.release();
            hierarquia.release();
            mascaraDebug.release();
            streamFrame.release();
            frameMetade.release();
            lut.release();
            lutBrilho = Double.NaN;
            lutGamma = Double.NaN;
            if (kernelMorph != null)
                kernelMorph.release();
            kernelMorph = null;
            kernelMorphTam = -1;
            if (roiMaskCache != null)
                roiMaskCache.release();
            roiMaskCache = null;
            roiMaskRegioes = null;
        }
    }

    private void aplicarConfigCamera() {
        erroConfig = "";
        try {
            PixelFormat formato = formatoYuyv ? PixelFormat.kYUYV : PixelFormat.kMJPEG;
            if (!camera.setVideoMode(formato, capturaW, capturaH, fps))
                erroConfig = "modo nao aceito: " + formato + " " + capturaW + "x" + capturaH + " " + fps + "fps";
        } catch (Throwable e) {
            erroConfig = "modo camera: " + e;
        }

        // cada ajuste separado: camera que nao suporta um nao bloqueia os outros.
        // Modo automatico recusado e ignorado; so avisa se um valor manual falhar.
        try {
            if (exposicao < 0)
                camera.setExposureAuto();
            else
                camera.setExposureManual(exposicao);
        } catch (Throwable e) {
            if (exposicao >= 0)
                erroConfig = "exposicao manual nao suportada";
        }
        try {
            if (balancoBranco < 0)
                camera.setWhiteBalanceAuto();
            else
                camera.setWhiteBalanceManual(balancoBranco);
        } catch (Throwable e) {
            if (balancoBranco >= 0)
                erroConfig = "balanco de branco manual nao suportado";
        }
        try {
            if (brilhoCamera >= 0)
                camera.setBrightness(brilhoCamera);
        } catch (Throwable e) {
            erroConfig = "brilho da camera nao suportado";
        }
    }

    /** Imprime no console os modos que a camera suporta e o modo em uso. */
    private void listarModos() {
        try {
            StringBuilder sb = new StringBuilder("[Visao] modos da camera:");
            for (VideoMode m : camera.enumerateVideoModes())
                sb.append("\n   ").append(m.pixelFormat).append(' ').append(m.width).append('x').append(m.height)
                        .append(' ').append(m.fps).append("fps");
            System.out.println(sb);
            atualizarStatusCamera();
            System.out.println("[Visao] em uso: " + modoCamera);
        } catch (Exception e) {
            System.out.println("[Visao] nao foi possivel listar os modos: " + e.getMessage());
        }
    }

    /** Processa um frame. Retorna a mascara de calibracao para o stream, ou null. */
    private Mat processarFrame(Mat frame, Mat corrigido, Mat hsv, Mat mask, Mat temp, Mat hierarquia,
            List<MatOfPoint> contornos, Mat mascaraDebug, boolean desenhar) {

        final int w = frame.cols();
        final int h = frame.rows();
        final VisionProfile p = perfil;

        if (p == null || p.getObjects().isEmpty()) {
            resultado = Resultado.vazio(w, h);
            return null;
        }

        final double[][] regs = regioesRoi;
        final Rect roi = calcularRoiUniao(regs, w, h);
        final Mat roiMask = obterMascaraRoi(regs, roi, w, h); // null = recorte ja e exato
        final Mat crop = frame.submat(roi);

        final List<Deteccao> lista = new ArrayList<>();
        final double centroXpx = w / 2.0 + centroOffsetXpx;
        final double centroYpx = h / 2.0 + centroOffsetYpx;
        final double escala = (larguraBaseCalibracao > 0) ? w / (double) larguraBaseCalibracao : 1.0;
        final double escalaArea = escala * escala;
        final String nomeMascara = desenhar ? streamMascara : null;
        final boolean desenhaRetangulo = desenhar && desenharDeteccoes && nomeMascara == null;
        boolean mascaraGerada = false;

        try {
            // brilho + gamma numa LUT (pulada se neutra)
            Mat fonte = crop;
            if (!p.isCorrecaoNeutra()) {
                atualizarLut(p.getBrightness(), p.getGamma());
                Core.LUT(crop, lut, corrigido);
                fonte = corrigido;
            }

            Imgproc.cvtColor(fonte, hsv, Imgproc.COLOR_BGR2HSV);

            for (VisionObject obj : p.getObjects()) {
                List<HsvRange> ranges = obj.getRanges();
                if (ranges.isEmpty())
                    continue;

                // mascara = OU de todas as faixas
                HsvRange r0 = ranges.get(0);
                Core.inRange(hsv, r0.getLower(), r0.getUpper(), mask);
                for (int i = 1; i < ranges.size(); i++) {
                    HsvRange ri = ranges.get(i);
                    Core.inRange(hsv, ri.getLower(), ri.getUpper(), temp);
                    Core.bitwise_or(mask, temp, mask);
                }

                if (roiMask != null)
                    Core.bitwise_and(mask, roiMask, mask);

                if (p.blurAtivo())
                    Imgproc.medianBlur(mask, mask, p.getBlurKernel());

                if (p.morphAtivo()) {
                    Mat k = obterKernel(p.getMorphKernel());
                    Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, k);
                    Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, k);
                }

                if (nomeMascara != null && !mascaraGerada && obj.getName().equalsIgnoreCase(nomeMascara)) {
                    Imgproc.cvtColor(mask, mascaraDebug, Imgproc.COLOR_GRAY2BGR);
                    mascaraGerada = true;
                }

                // so contornos externos, so vertices (modo mais leve)
                Imgproc.findContours(mask, contornos, hierarquia, Imgproc.RETR_EXTERNAL,
                        Imgproc.CHAIN_APPROX_SIMPLE);

                final double minA = obj.getMinArea() * escalaArea;
                final double maxA = obj.getMaxArea() * escalaArea;

                for (int i = 0; i < contornos.size(); i++) {
                    MatOfPoint c = contornos.get(i);
                    double area = Imgproc.contourArea(c);
                    if (area >= minA && area <= maxA) {
                        Rect b = Imgproc.boundingRect(c);
                        int x = b.x + roi.x;
                        int y = b.y + roi.y;
                        lista.add(new Deteccao(obj, x, y, b.width, b.height, area, centroXpx, centroYpx));
                        // retangulo = o mesmo usado nas posicoes (x, y, centro)
                        if (desenhaRetangulo) {
                            Imgproc.rectangle(frame, new Point(x, y), new Point(x + b.width - 1, y + b.height - 1),
                                    obj.getDrawColor(), espessuraContorno);
                        }
                    }
                    c.release();
                }
                contornos.clear();
            }
        } finally {
            crop.release();
        }

        Deteccao alvo = escolherAlvoPrioritario(lista, centroXpx, w);
        if (desenhar && nomeMascara == null && alvo != null)
            desenharCruz(frame, alvo);
        Resultado novo;
        if (alvo != null && alvo.largura > 0) {
            double prof = (kProfundidade * escala) / alvo.largura;
            double f = focalPixels * escala;
            novo = new Resultado(Collections.unmodifiableList(lista), alvo, prof,
                    alvo.xBorda * prof / f, alvo.yBorda * prof / f,
                    alvo.xCentro * prof / f, alvo.yCentro * prof / f,
                    w, h, Timer.getFPGATimestamp(), tCaptura);
        } else {
            novo = new Resultado(Collections.unmodifiableList(lista), null, -1, -1, -1, -1, -1, w, h,
                    Timer.getFPGATimestamp(), tCaptura);
        }
        // se pausou durante o frame, nao publica alvo velho
        resultado = pausado ? Resultado.vazio(w, h) : novo;

        return (desenhar && mascaraGerada) ? mascaraDebug : null;
    }

    /**
     * Alvo prioritario: o MESMO objeto do frame anterior (o mais perto da posicao dele),
     * para nao pular sozinho entre objetos. Sem alvo travado, escolhe pelo modo.
     * Atende o pedido de "proximo alvo" (esquerda -> direita, volta ao primeiro).
     */
    private Deteccao escolherAlvoPrioritario(List<Deteccao> lista, double centroXpx, int w) {
        // candidatos: respeitam o seletor por nome, da esquerda para a direita
        String seletor = seletorAlvo;
        List<Deteccao> candidatos = new ArrayList<>();
        for (Deteccao d : lista) {
            if (seletor.isEmpty() || d.nome.equalsIgnoreCase(seletor))
                candidatos.add(d);
        }
        candidatos.sort((a, b) -> Integer.compare(a.xCentro, b.xCentro));
        totalCandidatos = candidatos.size();

        if (pedirSoltarAlvo) {
            pedirSoltarAlvo = false;
            rastreando = false;
        }

        // 1. segue o objeto travado
        Deteccao alvo = null;
        if (rastreando) {
            alvo = maisProximo(candidatos, rastreioX, rastreioY, RAIO_RASTREIO * w);
            if (alvo == null && ++framesSemAlvo > FRAMES_PERDA)
                rastreando = false; // sumiu por muito tempo: esquece
        }
        // 2. nada travado: escolhe pelo modo (CENTRO, ESQUERDA...)
        if (!rastreando)
            alvo = escolherAlvo(candidatos, "", modoAlvo, centroXpx);
        // 3. botao "proximo alvo": o seguinte da esquerda para a direita
        if (pedirProximoAlvo) {
            pedirProximoAlvo = false;
            if (!candidatos.isEmpty()) {
                int atual = (alvo == null) ? -1 : candidatos.indexOf(alvo);
                alvo = candidatos.get((atual + 1) % candidatos.size());
            }
        }

        if (alvo != null) {
            rastreando = true;
            rastreioX = alvo.xCentro;
            rastreioY = alvo.yCentro;
            framesSemAlvo = 0;
            indiceAlvo = candidatos.indexOf(alvo) + 1;
        } else {
            indiceAlvo = 0;
        }
        return alvo;
    }

    /** Deteccao mais perto de (x, y) cartesiano, ate a distancia maxima. null se nenhuma. */
    private static Deteccao maisProximo(List<Deteccao> lista, int x, int y, double maxDist) {
        Deteccao melhor = null;
        double melhorD2 = maxDist * maxDist;
        for (Deteccao d : lista) {
            double dx = d.xCentro - x, dy = d.yCentro - y;
            double d2 = dx * dx + dy * dy;
            if (d2 <= melhorD2) {
                melhorD2 = d2;
                melhor = d;
            }
        }
        return melhor;
    }

    /** Cruz verde fluorescente no centro do alvo prioritario (com contorno preto para destacar). */
    private static void desenharCruz(Mat frame, Deteccao alvo) {
        int cx = alvo.x + alvo.largura / 2;
        int cy = alvo.y + alvo.altura / 2;
        int braco = Math.max(6, Math.min(alvo.largura, alvo.altura) / 3);
        Point h1 = new Point(cx - braco, cy), h2 = new Point(cx + braco, cy);
        Point v1 = new Point(cx, cy - braco), v2 = new Point(cx, cy + braco);
        Imgproc.line(frame, h1, h2, PRETO, 4);
        Imgproc.line(frame, v1, v2, PRETO, 4);
        Imgproc.line(frame, h1, h2, VERDE_FLUOR, 2);
        Imgproc.line(frame, v1, v2, VERDE_FLUOR, 2);
    }

    private static Deteccao escolherAlvo(List<Deteccao> lista, String seletor, ModoAlvo modo, double centroXpx) {
        boolean filtra = seletor != null && !seletor.isEmpty();
        Deteccao melhor = null;
        double melhorScore = Double.MAX_VALUE;
        for (int i = 0; i < lista.size(); i++) {
            Deteccao d = lista.get(i);
            if (filtra && !d.nome.equalsIgnoreCase(seletor))
                continue;
            double cx = d.x + d.largura / 2.0;
            double score;
            switch (modo) {
            case ESQUERDA:
                score = cx;
                break;
            case DIREITA:
                score = -cx;
                break;
            case MAIOR:
                score = -d.area;
                break;
            case CENTRO:
            default:
                score = Math.abs(cx - centroXpx);
                break;
            }
            if (score < melhorScore) {
                melhorScore = score;
                melhor = d;
            }
        }
        return melhor;
    }

    private void enviarStream(Mat fonte, Mat streamFrame) {
        try {
            if (fonte.empty())
                return;
            int sw = streamW, sh = streamH;
            boolean redimensiona = sw > 0 && sh > 0 && (fonte.cols() != sw || fonte.rows() != sh);
            if (redimensiona) {
                Imgproc.resize(fonte, streamFrame, new Size(sw, sh), 0, 0, Imgproc.INTER_NEAREST);
                output.putFrame(streamFrame);
            } else {
                output.putFrame(fonte);
            }
        } catch (Throwable e) {
            ultimoErro = "stream: " + e;
        }
    }

    /** LUT de brilho+gamma, remontada so quando os valores mudam. */
    private void atualizarLut(double brilho, double gamma) {
        if (brilho == lutBrilho && gamma == lutGamma)
            return;
        byte[] tabela = new byte[256];
        for (int i = 0; i < 256; i++) {
            double v = Math.max(0, Math.min(255, i + brilho));
            v = Math.pow(v / 255.0, gamma) * 255.0;
            tabela[i] = (byte) (int) Math.round(Math.max(0, Math.min(255, v)));
        }
        lut.put(0, 0, tabela);
        lutBrilho = brilho;
        lutGamma = gamma;
    }

    private Mat obterKernel(int tam) {
        if (kernelMorph == null || kernelMorphTam != tam) {
            if (kernelMorph != null)
                kernelMorph.release();
            kernelMorph = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(tam, tam));
            kernelMorphTam = tam;
        }
        return kernelMorph;
    }

    /** Retangulo que cobre todas as regioes de ROI. */
    private static Rect calcularRoiUniao(double[][] regs, int w, int h) {
        if (regs == null || regs.length == 0)
            return new Rect(0, 0, w, h);
        double minX = 1, minY = 1, maxX = 0, maxY = 0;
        for (double[] r : regs) {
            minX = Math.min(minX, r[0]);
            minY = Math.min(minY, r[1]);
            maxX = Math.max(maxX, r[2]);
            maxY = Math.max(maxY, r[3]);
        }
        int x0 = clamp((int) Math.floor(minX * w), 0, w - 1);
        int y0 = clamp((int) Math.floor(minY * h), 0, h - 1);
        int x1 = clamp((int) Math.ceil(maxX * w), x0 + 1, w);
        int y1 = clamp((int) Math.ceil(maxY * h), y0 + 1, h);
        return new Rect(x0, y0, x1 - x0, y1 - y0);
    }

    /** Mascara dos blocos ativos (so com 2+ regioes), cacheada. */
    private Mat obterMascaraRoi(double[][] regs, Rect uniao, int w, int h) {
        if (regs == null || regs.length <= 1)
            return null;
        if (roiMaskCache != null && regs == roiMaskRegioes && w == roiMaskW && h == roiMaskH)
            return roiMaskCache;

        if (roiMaskCache == null)
            roiMaskCache = new Mat();
        roiMaskCache.create(uniao.height, uniao.width, CvType.CV_8UC1);
        roiMaskCache.setTo(new Scalar(0));
        for (double[] r : regs) {
            int bx0 = clamp((int) Math.floor(r[0] * w) - uniao.x, 0, uniao.width);
            int by0 = clamp((int) Math.floor(r[1] * h) - uniao.y, 0, uniao.height);
            int bx1 = clamp((int) Math.ceil(r[2] * w) - uniao.x, 0, uniao.width);
            int by1 = clamp((int) Math.ceil(r[3] * h) - uniao.y, 0, uniao.height);
            if (bx1 > bx0 && by1 > by0)
                Imgproc.rectangle(roiMaskCache, new Point(bx0, by0), new Point(bx1 - 1, by1 - 1), BRANCO, -1);
        }
        roiMaskRegioes = regs;
        roiMaskW = w;
        roiMaskH = h;
        return roiMaskCache;
    }

    private static int clamp(int v, int min, int max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double clamp01(double v) {
        return Math.max(0, Math.min(1, v));
    }

    private static void dormir(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
