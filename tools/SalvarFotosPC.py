"""
Salva no PC (Area de Trabalho/FotosRobo) a imagem do stream, igual ao Elastic,
sempre que o botao BACK do controle e apertado.

Uso:
    pip install pyntcore opencv-python
    python SalvarFotosPC.py            (robo)
    python SalvarFotosPC.py localhost  (simulacao)
"""
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import ntcore

EQUIPE = 10019
STREAM = "VisaoHSV"
PASTA = Path.home() / "Desktop" / "FotosRobo"

ultimo_quadro = None
trava = threading.Lock()


def ler_stream(url):
    global ultimo_quadro
    while True:
        cap = cv2.VideoCapture(url)
        while cap.isOpened():
            ok, quadro = cap.read()
            if not ok:
                break
            with trava:
                ultimo_quadro = quadro
        cap.release()
        time.sleep(1)


def url_do_stream(nt):
    streams = nt.getTable("CameraPublisher/" + STREAM).getEntry("streams").getStringArray([])
    for s in streams:
        if s.startswith("mjpg:"):
            return s[5:]
    return None


def main():
    nt = ntcore.NetworkTableInstance.getDefault()
    nt.startClient4("fotos-pc")
    if len(sys.argv) > 1:
        nt.setServer(sys.argv[1])
    else:
        nt.setServerTeam(EQUIPE)
    disparo = nt.getTable("SmartDashboard").getEntry("Visao/Foto/Disparo")

    PASTA.mkdir(parents=True, exist_ok=True)
    print("Esperando o robo... fotos em", PASTA)

    url = None
    while url is None:
        time.sleep(0.5)
        url = url_do_stream(nt)
    print("Stream:", url)
    threading.Thread(target=ler_stream, args=(url,), daemon=True).start()

    anterior = disparo.getDouble(0)
    while True:
        time.sleep(0.05)
        atual = disparo.getDouble(0)
        if atual == anterior:
            continue
        anterior = atual
        with trava:
            quadro = None if ultimo_quadro is None else ultimo_quadro.copy()
        if quadro is None:
            print("Sem imagem do stream.")
            continue
        nome = PASTA / datetime.now().strftime("foto_%Y%m%d_%H%M%S_%f.jpg")
        cv2.imwrite(str(nome), quadro, [cv2.IMWRITE_JPEG_QUALITY, 100])
        print("Salva:", nome.name)


if __name__ == "__main__":
    main()
