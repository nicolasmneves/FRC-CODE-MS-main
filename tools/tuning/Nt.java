import java.io.*;
import java.net.URI;
import java.net.http.*;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.*;

/**
 * Cliente NT4 minimo (sem dependencias) para ler/escrever NetworkTables do robo pelo notebook.
 * Roda com o JDK do WPILib:  java Nt.java <host> <comando> [args]
 *
 *   dump  <segundos> <prefixo...>              mostra o ultimo valor de cada topico
 *   log   <segundos> <arquivo.csv> <prefixo...> grava todas as amostras (t_s,topico,valor)
 *   set   <topico> <double>                     publica um double (ex.: /Tuning/Angle/kP 60)
 *   setb  <topico> <true|false>                 publica um boolean
 */
public class Nt {
  static final Map<Long, String> names = new ConcurrentHashMap<>();
  static final Map<Long, String> types = new ConcurrentHashMap<>();
  static final Map<String, Object> last = new ConcurrentSkipListMap<>();
  static PrintWriter csv;
  static long t0 = -1;

  public static void main(String[] a) throws Exception {
    String host = a[0], cmd = a[1];
    HttpClient hc = HttpClient.newHttpClient();
    CountDownLatch open = new CountDownLatch(1);
    WebSocket ws = hc.newWebSocketBuilder()
        .subprotocols("v4.1.networktables.first.wpi.edu", "networktables.first.wpi.edu")
        .buildAsync(URI.create("ws://" + host + ":5810/nt/claude-tuning"), new WebSocket.Listener() {
          StringBuilder sb = new StringBuilder();
          ByteArrayOutputStream bb = new ByteArrayOutputStream();
          public void onOpen(WebSocket w) { open.countDown(); w.request(1); }
          public CompletionStage<?> onText(WebSocket w, CharSequence d, boolean lastPart) {
            sb.append(d);
            if (lastPart) { try { handleText(sb.toString()); } catch (Throwable e) { System.err.println("ERRO txt: " + e); } sb.setLength(0); }
            w.request(1); return null;
          }
          public CompletionStage<?> onBinary(WebSocket w, ByteBuffer d, boolean lastPart) {
            byte[] b = new byte[d.remaining()]; d.get(b); bb.write(b, 0, b.length);
            if (lastPart) { try { handleBinary(bb.toByteArray()); } catch (Throwable e) { System.err.println("ERRO bin: " + e); } bb.reset(); }
            w.request(1); return null;
          }
          public void onError(WebSocket w, Throwable e) { System.err.println("ERRO WS: " + e); }
        }).get(5, TimeUnit.SECONDS);
    open.await(5, TimeUnit.SECONDS);

    switch (cmd) {
      case "dump", "log" -> {
        double secs = Double.parseDouble(a[2]);
        int first = 3;
        if (cmd.equals("log")) { csv = new PrintWriter(new FileWriter(a[3]), true); csv.println("t,topic,value"); first = 4; }
        StringBuilder topics = new StringBuilder();
        for (int i = first; i < a.length; i++) topics.append(i > first ? "," : "").append('"').append(a[i]).append('"');
        ws.sendText("[{\"method\":\"subscribe\",\"params\":{\"topics\":[" + topics +
            "],\"subuid\":1,\"options\":{\"prefix\":true,\"periodic\":0.02,\"all\":true}}}]", true).get();
        Thread.sleep((long) (secs * 1000));
        if (csv != null) { csv.close(); System.out.println("amostras gravadas em " + a[3]); }
        else last.forEach((k, v) -> System.out.println(k + " = " + v));
      }
      case "set", "setb" -> {
        boolean isB = cmd.equals("setb");
        // assina o proprio topico para descobrir o relogio do servidor antes de publicar
        ws.sendText("[{\"method\":\"subscribe\",\"params\":{\"topics\":[\"" + a[2] + "\",\"/FMSInfo/FMSControlData\"],\"subuid\":9,\"options\":{}}}]", true).get();
        Thread.sleep(500);
        ws.sendText("[{\"method\":\"publish\",\"params\":{\"name\":\"" + a[2] + "\",\"pubuid\":7,\"type\":\"" +
            (isB ? "boolean" : "double") + "\",\"properties\":{}}}]", true).get();
        Thread.sleep(200);
        ByteArrayOutputStream o = new ByteArrayOutputStream();
        o.write(0x94); o.write(7); writeTs(o); // [pubuid, ts(relogio do servidor), type, value]
        if (isB) { o.write(0); o.write(Boolean.parseBoolean(a[3]) ? 0xc3 : 0xc2); }
        else { o.write(1); o.write(0xcb); o.write(ByteBuffer.allocate(8).putDouble(Double.parseDouble(a[3])).array()); }
        ws.sendBinary(ByteBuffer.wrap(o.toByteArray()), true).get();
        Thread.sleep(400);
        System.out.println("ok " + a[2] + " = " + a[3]);
      }
      case "step" -> step(ws, a);
      case "hstep" -> hstep(ws, a);
      case "hguard" -> hguard(ws, a);
      default -> System.out.println("comando desconhecido");
    }
    ws.sendClose(WebSocket.NORMAL_CLOSURE, "").get(2, TimeUnit.SECONDS);
    System.exit(0);
  }

  // ================= TESTE DE DEGRAU DA ROTACAO =================
  // step <kP> <kD> <kS> <kV> <angA> <angB> <ciclos> [segPorDegrau]
  // Robo precisa estar ENABLED em TEST (SteerTuningCommand rodando) e SUSPENSO (rodas fora do chao).
  static final List<double[]> amostras = Collections.synchronizedList(new ArrayList<>()); // [t, a0..a3]
  static volatile long angleTopic = -999;

  static void pub(WebSocket ws, String name, int uid) throws Exception {
    ws.sendText("[{\"method\":\"publish\",\"params\":{\"name\":\"" + name + "\",\"pubuid\":" + uid +
        ",\"type\":\"double\",\"properties\":{}}}]", true).get();
  }

  // Relogio do servidor (us), estimado pelo ultimo valor recebido. O NT descarta valores com
  // carimbo mais antigo que o atual, entao publicar com ts=0 nao sobrescreve o que o robo escreveu.
  static volatile long serverTs = 0, serverTsLocalNs = 0;

  static void writeTs(ByteArrayOutputStream o) {
    if (serverTs == 0) { o.write(0); return; }
    long ts = serverTs + (System.nanoTime() - serverTsLocalNs) / 1000 + 1000;
    o.write(0xcf);
    o.write(ByteBuffer.allocate(8).putLong(ts).array(), 0, 8);
  }

  static void send(WebSocket ws, int uid, double v) throws Exception {
    ByteArrayOutputStream o = new ByteArrayOutputStream();
    o.write(0x94); o.write(uid); writeTs(o); o.write(1); o.write(0xcb);
    o.write(ByteBuffer.allocate(8).putDouble(v).array());
    ws.sendBinary(ByteBuffer.wrap(o.toByteArray()), true).get();
  }

  static void step(WebSocket ws, String[] a) throws Exception {
    double p = Double.parseDouble(a[2]), d = Double.parseDouble(a[3]), s = Double.parseDouble(a[4]), v = Double.parseDouble(a[5]);
    double A = Double.parseDouble(a[6]), B = Double.parseDouble(a[7]);
    int ciclos = Integer.parseInt(a[8]);
    double seg = a.length > 9 ? Double.parseDouble(a[9]) : 1.5;
    String[] nomes = {"kP", "kD", "kS", "kV", "SetpointDeg"};
    for (int i = 0; i < nomes.length; i++) pub(ws, "/Tuning/Steer/" + nomes[i], 10 + i);
    ws.sendText("[{\"method\":\"subscribe\",\"params\":{\"topics\":[\"/Tuning/Steer/AngleDeg\"],\"subuid\":2,\"options\":{\"periodic\":0.005,\"all\":true}}}]", true).get();
    Thread.sleep(300);
    send(ws, 10, p); send(ws, 11, d); send(ws, 12, s); send(ws, 13, v);
    send(ws, 14, A);
    Thread.sleep(1500);
    if (amostras.isEmpty()) { System.out.println("SEM DADOS: o robo esta habilitado em modo TEST?"); return; }
    List<double[]> degraus = new ArrayList<>(); // [tInicio, de, para]
    double atual = A;
    for (int c = 0; c < 2 * ciclos; c++) {
      double alvo = (atual == A) ? B : A;
      degraus.add(new double[]{System.nanoTime() / 1e9, atual, alvo});
      send(ws, 14, alvo);
      atual = alvo;
      Thread.sleep((long) (seg * 1000));
    }
    String[] mod = {"FL", "FR", "BL", "BR"};
    System.out.printf("kP=%.1f kD=%.2f kS=%.2f kV=%.2f  degraus %.0f<->%.0f%n", p, d, s, v, A, B);
    System.out.println("modulo  subida10-90(ms)  overshoot(graus)  acomod.2graus(ms)  erroFinal(graus)  oscilacao(graus pp)");
    double[] somaR = new double[4], somaO = new double[4], somaS = new double[4], somaE = new double[4], somaOsc = new double[4];
    List<double[]> snap; synchronized (amostras) { snap = new ArrayList<>(amostras); }
    for (double[] dg : degraus) {
      double t0 = dg[0], de = dg[1], para = dg[2], amp = para - de;
      for (int m = 0; m < 4; m++) {
        double t10 = Double.NaN, t90 = Double.NaN, over = 0, tSettle = 0, fim = 0; int nFim = 0;
        double minFim = 1e9, maxFim = -1e9;
        for (double[] sa : snap) {
          double t = sa[0] - t0; if (t < 0 || t > seg) continue;
          double ang = unwrapNear(sa[1 + m], para);
          double frac = (ang - de) / amp;
          if (Double.isNaN(t10) && frac >= 0.1) t10 = t;
          if (Double.isNaN(t90) && frac >= 0.9) t90 = t;
          over = Math.max(over, (ang - para) * Math.signum(amp));
          if (Math.abs(ang - para) > 2) tSettle = t;
          if (t > seg - 0.4) { fim += ang - para; nFim++; minFim = Math.min(minFim, ang); maxFim = Math.max(maxFim, ang); }
        }
        double rise = (t90 - t10) * 1000, err = nFim > 0 ? fim / nFim : Double.NaN, osc = nFim > 0 ? maxFim - minFim : Double.NaN;
        somaR[m] += rise; somaO[m] += over; somaS[m] += tSettle * 1000; somaE[m] += Math.abs(err); somaOsc[m] += osc;
      }
    }
    String csvPath = System.getenv("STEP_CSV");
    if (csvPath != null) {
      try (PrintWriter w = new PrintWriter(new FileWriter(csvPath))) {
        double tb = degraus.get(0)[0];
        w.println("t,alvo,FL,FR,BL,BR");
        for (double[] sa : snap) {
          double alvo = A;
          for (double[] dg : degraus) if (sa[0] >= dg[0]) alvo = dg[2];
          w.printf(Locale.ROOT, "%.4f,%.0f,%.2f,%.2f,%.2f,%.2f%n", sa[0] - tb, alvo,
              unwrapNear(sa[1], alvo), unwrapNear(sa[2], alvo), unwrapNear(sa[3], alvo), unwrapNear(sa[4], alvo));
        }
      }
    }
    int n = degraus.size();
    for (int m = 0; m < 4; m++)
      System.out.printf("%-6s  %15.0f  %16.2f  %17.0f  %16.2f  %19.2f%n", mod[m], somaR[m] / n, somaO[m] / n, somaS[m] / n, somaE[m] / n, somaOsc[m] / n);
    send(ws, 14, A);
    Thread.sleep(300);
  }

  // ================= TESTE DE DEGRAU DO RUMO =================
  // hstep <kP> <kD> <deltaA> <deltaB> <ciclos> [segPorDegrau]
  // Robo no chao, Test mode com /Tuning/Mode = 1. Os alvos sao relativos ao rumo inicial
  // (ex.: 0 e 90 = gira 90 graus e volta), para o cabo nao enrolar.
  static final List<double[]> rumo = Collections.synchronizedList(new ArrayList<>()); // [t, graus]
  static final List<double[]> bateria = Collections.synchronizedList(new ArrayList<>()); // [t, volts]
  static final List<Double> brownouts = Collections.synchronizedList(new ArrayList<>());

  static void hstep(WebSocket ws, String[] a) throws Exception {
    double p = Double.parseDouble(a[2]), d = Double.parseDouble(a[3]);
    double dA = Double.parseDouble(a[4]), dB = Double.parseDouble(a[5]);
    int ciclos = Integer.parseInt(a[6]);
    double seg = a.length > 7 ? Double.parseDouble(a[7]) : 3.0;
    pub(ws, "/Tuning/Heading/kP", 20); pub(ws, "/Tuning/Heading/kD", 21); pub(ws, "/Tuning/Heading/AlvoGraus", 22);
    pub(ws, "/Tuning/Heading/GiroSubida", 23); pub(ws, "/Tuning/Heading/GiroDescida", 24);
    ws.sendText("[{\"method\":\"subscribe\",\"params\":{\"topics\":[\"/Tuning/Heading/AtualGraus\",\"/SmartDashboard/Robo/BateriaV\",\"/SmartDashboard/Robo/Brownout\"],\"subuid\":3,\"options\":{\"periodic\":0.005,\"all\":true}}}]", true).get();
    Thread.sleep(800);
    if (rumo.isEmpty()) { System.out.println("SEM DADOS: Test mode habilitado com /Tuning/Mode = 1?"); return; }
    double base = rumo.get(rumo.size() - 1)[1];
    send(ws, 20, p); send(ws, 21, d);
    // limites opcionais do giro: GIRO_SUBIDA=12 GIRO_DESCIDA=40 ./nt.sh hstep ...
    String gs = System.getenv("GIRO_SUBIDA"), gd = System.getenv("GIRO_DESCIDA");
    if (gs != null) send(ws, 23, Double.parseDouble(gs));
    if (gd != null) send(ws, 24, Double.parseDouble(gd));
    double A = base + dA, B = base + dB;
    send(ws, 22, A);
    Thread.sleep((long) (seg * 1000));
    List<double[]> degraus = new ArrayList<>();
    double atual = A;
    for (int c = 0; c < 2 * ciclos; c++) {
      double alvo = (atual == A) ? B : A;
      degraus.add(new double[]{System.nanoTime() / 1e9, atual, alvo});
      send(ws, 22, alvo);
      atual = alvo;
      Thread.sleep((long) (seg * 1000));
    }
    send(ws, 22, base);
    Thread.sleep(300);
    List<double[]> snap; synchronized (rumo) { snap = new ArrayList<>(rumo); }
    List<double[]> bat; synchronized (bateria) { bat = new ArrayList<>(bateria); }
    System.out.printf("kP=%.3f kD=%.3f  degraus %+.0f<->%+.0f graus  limites giro: subida %s descida %s%n", p, d, dA, dB,
        gs == null ? "(atual)" : gs, gd == null ? "(atual)" : gd);
    System.out.println("degrau     subida10-90(ms)  overshoot(graus)  acomod.2graus(ms)  erroFinal(graus)  trancos  bateriaMin(V)  brownouts");
    double sR = 0, sO = 0, sS = 0, sE = 0, sJ = 0, vMin = 99; int sB = 0;
    String csvPath = System.getenv("STEP_CSV");
    PrintWriter w = csvPath != null ? new PrintWriter(new FileWriter(csvPath)) : null;
    if (w != null) w.println("t,alvo,graus,vel_graus_s,bateria");
    for (double[] dg : degraus) {
      double t0 = dg[0], de = dg[1], para = dg[2], amp = para - de;
      double t10 = Double.NaN, t90 = Double.NaN, over = 0, tSettle = 0, ultimo = Double.NaN;
      // "trancos": durante o giro, quantas vezes a velocidade angular cai >= 40 graus/s em relacao
      // ao pico recente e depois volta a subir (o robo "engasga" e retoma).
      double velPico = 0, velVale = 0; boolean emQueda = false; int trancos = 0;
      double tAnt = Double.NaN, angAnt = Double.NaN;
      for (double[] sa : snap) {
        double t = sa[0] - t0; if (t < 0 || t > seg) continue;
        double ang = unwrapNear(sa[1], para);
        double frac = (ang - de) / amp;
        if (Double.isNaN(t10) && frac >= 0.1) t10 = t;
        if (Double.isNaN(t90) && frac >= 0.9) t90 = t;
        over = Math.max(over, (ang - para) * Math.signum(amp));
        if (Math.abs(ang - para) > 2) tSettle = t;
        ultimo = ang - para;
        double vel = Double.NaN;
        if (!Double.isNaN(tAnt) && t - tAnt > 0.015) {
          vel = (ang - angAnt) / (t - tAnt) * Math.signum(amp); // positivo = indo para o alvo
          if (frac > 0.05 && frac < 0.85) {
            if (!emQueda) { velPico = Math.max(velPico, vel); if (vel < velPico - 40) { emQueda = true; velVale = vel; } }
            else { velVale = Math.min(velVale, vel); if (vel > velVale + 40) { trancos++; emQueda = false; velPico = vel; } }
          }
          tAnt = t; angAnt = ang;
        } else if (Double.isNaN(tAnt)) { tAnt = t; angAnt = ang; }
        if (w != null) w.printf(Locale.ROOT, "%.4f,%.1f,%.2f,%s,%n", sa[0] - degraus.get(0)[0], para - base, ang - base,
            Double.isNaN(vel) ? "" : String.format(Locale.ROOT, "%.0f", vel));
      }
      double vm = 99;
      for (double[] b : bat) if (b[0] >= t0 && b[0] <= t0 + seg) vm = Math.min(vm, b[1]);
      int nb = 0;
      synchronized (brownouts) { for (double tb : brownouts) if (tb >= t0 && tb <= t0 + seg) nb++; }
      double rise = (t90 - t10) * 1000;
      System.out.printf(Locale.ROOT, "%+5.0f->%+4.0f  %15.0f  %16.2f  %17.0f  %16.2f  %7d  %13.2f  %9d%n", de - base, para - base, rise, over, tSettle * 1000, ultimo, trancos, vm, nb);
      sR += rise; sO += over; sS += tSettle * 1000; sE += Math.abs(ultimo); sJ += trancos; vMin = Math.min(vMin, vm); sB += nb;
    }
    if (w != null) w.close();
    int n = degraus.size();
    System.out.printf(Locale.ROOT, "MEDIA       %15.0f  %16.2f  %17.0f  %16.2f  %7.1f  %13.2f  %9d%n", sR / n, sO / n, sS / n, sE / n, sJ / n, vMin, sB);
  }

  // ================= GIRO DE VERIFICACAO COM TRAVA =================
  // hguard <kP> <delta>: pede base+delta com kP baixo. Se o erro CRESCER (giro invertido),
  // zera kP e kD na hora e avisa. Use antes de qualquer tuning depois de mexer nos modulos.
  static void hguard(WebSocket ws, String[] a) throws Exception {
    double p = Double.parseDouble(a[2]), delta = Double.parseDouble(a[3]);
    pub(ws, "/Tuning/Heading/kP", 20); pub(ws, "/Tuning/Heading/kD", 21); pub(ws, "/Tuning/Heading/AlvoGraus", 22);
    ws.sendText("[{\"method\":\"subscribe\",\"params\":{\"topics\":[\"/Tuning/Heading/AtualGraus\"],\"subuid\":3,\"options\":{\"periodic\":0.005,\"all\":true}}}]", true).get();
    Thread.sleep(800);
    if (rumo.isEmpty()) { System.out.println("SEM DADOS: Test mode habilitado com /Tuning/Mode = 1?"); return; }
    double base = rumo.get(rumo.size() - 1)[1], alvo = base + delta;
    send(ws, 21, 0); send(ws, 22, alvo); send(ws, 20, p);
    long fim = System.nanoTime() + 3_000_000_000L;
    double pior = 0, ultimo = Double.NaN;
    while (System.nanoTime() < fim) {
      double atual = rumo.get(rumo.size() - 1)[1];
      double err = unwrapNear(atual, alvo) - alvo;
      ultimo = err; pior = Math.max(pior, Math.abs(err));
      if (Math.abs(err) > Math.abs(delta) + 10) {
        send(ws, 20, 0); send(ws, 21, 0);
        System.out.printf(Locale.ROOT, "DISPARO: erro %.1f graus (pedido %.0f). kP zerado.%n", err, delta);
        return;
      }
      Thread.sleep(10);
    }
    System.out.printf(Locale.ROOT, "OK: pedido %+.0f graus, erro final %.2f graus (pior %.1f). Giro no sentido certo.%n", delta, ultimo, pior);
  }

  static double unwrapNear(double ang, double ref) {
    while (ang - ref > 180) ang -= 360;
    while (ang - ref < -180) ang += 360;
    return ang;
  }

  static void handleText(String s) {
    // parse simples dos anuncios: {"method":"announce","params":{"name":"..","id":N,"type":".."...}}
    int i = 0;
    while ((i = s.indexOf("\"announce\"", i)) >= 0) {
      String n = field(s, "\"name\"", i), id = field(s, "\"id\"", i), t = field(s, "\"type\"", i);
      if (n != null && id != null) { names.put(Long.parseLong(id), n); types.put(Long.parseLong(id), t); }
      i++;
    }
  }

  static String field(String s, String key, int from) {
    int k = s.indexOf(key, from); if (k < 0) return null;
    int c = s.indexOf(':', k) + 1;
    while (s.charAt(c) == ' ') c++;
    if (s.charAt(c) == '"') { int e = s.indexOf('"', c + 1); return s.substring(c + 1, e); }
    int e = c; while (e < s.length() && "-0123456789".indexOf(s.charAt(e)) >= 0) e++;
    return s.substring(c, e);
  }

  static void handleBinary(byte[] b) {
    Mp m = new Mp(b);
    while (m.p < b.length) {
      Object o = m.read();
      if (!(o instanceof List<?> l) || l.size() < 4) continue;
      long id = ((Number) l.get(0)).longValue();
      if (id < 0) continue; // sincronizacao de tempo
      long ts = ((Number) l.get(1)).longValue();
      if (ts > serverTs) { serverTs = ts; serverTsLocalNs = System.nanoTime(); }
      String n = names.getOrDefault(id, "#" + id);
      Object v = l.get(3);
      if (n.equals("/Tuning/Steer/AngleDeg") && v instanceof List<?> arr && arr.size() >= 4) {
        double[] sa = new double[5];
        sa[0] = System.nanoTime() / 1e9;
        for (int i = 0; i < 4; i++) sa[1 + i] = ((Number) arr.get(i)).doubleValue();
        amostras.add(sa);
      }
      if (n.equals("/Tuning/Heading/AtualGraus") && v instanceof Number num)
        rumo.add(new double[]{System.nanoTime() / 1e9, num.doubleValue()});
      if (n.equals("/SmartDashboard/Robo/BateriaV") && v instanceof Number num)
        bateria.add(new double[]{System.nanoTime() / 1e9, num.doubleValue()});
      if (n.equals("/SmartDashboard/Robo/Brownout") && Boolean.TRUE.equals(v))
        brownouts.add(System.nanoTime() / 1e9);
      if (v instanceof List<?> lst) v = lst.toString();
      last.put(n, v);
      if (csv != null) {
        if (t0 < 0) t0 = ts;
        synchronized (Nt.class) { csv.println(((ts - t0) / 1e6) + "," + n + "," + v); }
      }
    }
  }

  /** decodificador MessagePack minimo */
  static class Mp {
    final byte[] b; int p = 0;
    Mp(byte[] b) { this.b = b; }
    int u8() { return b[p++] & 0xff; }
    long un(int n) { long v = 0; for (int i = 0; i < n; i++) v = (v << 8) | u8(); return v; }
    Object read() {
      int t = u8();
      if (t <= 0x7f) return (long) t;
      if (t >= 0xe0) return (long) (byte) t;
      if ((t & 0xf0) == 0x90) return arr(t & 0x0f);
      if ((t & 0xf0) == 0x80) return map(t & 0x0f);
      if ((t & 0xe0) == 0xa0) return str(t & 0x1f);
      switch (t) {
        case 0xc0: return null;
        case 0xc2: return false;
        case 0xc3: return true;
        case 0xc4: { int n = u8(); p += n; return "<bin>"; }
        case 0xc5: { int n = (int) un(2); p += n; return "<bin>"; }
        case 0xc6: { int n = (int) un(4); p += n; return "<bin>"; }
        case 0xca: return (double) Float.intBitsToFloat((int) un(4));
        case 0xcb: return Double.longBitsToDouble(un(8));
        case 0xcc: return un(1);
        case 0xcd: return un(2);
        case 0xce: return un(4);
        case 0xcf: return un(8);
        case 0xd0: return (long) (byte) u8();
        case 0xd1: return (long) (short) un(2);
        case 0xd2: return (long) (int) un(4);
        case 0xd3: return un(8);
        case 0xd9: return str(u8());
        case 0xda: return str((int) un(2));
        case 0xdb: return str((int) un(4));
        case 0xdc: return arr((int) un(2));
        case 0xdd: return arr((int) un(4));
        case 0xde: return map((int) un(2));
        case 0xdf: return map((int) un(4));
        default: throw new IllegalStateException("msgpack tipo " + Integer.toHexString(t));
      }
    }
    List<Object> arr(int n) { List<Object> l = new ArrayList<>(n); for (int i = 0; i < n; i++) l.add(read()); return l; }
    Object map(int n) { for (int i = 0; i < 2 * n; i++) read(); return "<map>"; }
    String str(int n) { String s = new String(b, p, n, StandardCharsets.UTF_8); p += n; return s; }
  }
}
