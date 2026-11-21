#!/usr/bin/env bash
# Compila / faz deploy do projeto com JDK 17 (GradleRIO 2026).
# Uso: build_deploy.sh <JDK17_HOME> <tarefa gradle...>   ex.: build_deploy.sh /c/jdk17 build   |   ... deploy
# Se o projeto estiver dentro do OneDrive (o GradleRIO recusa), espelha para fora antes de compilar;
# fora do OneDrive (ex.: C:\FRC\FRC-CODE-MS) compila direto na pasta.
# Com a WPILib 2026 instalada, prefira o VS Code 2026 (Ctrl+Shift+P > "Deploy Robot Code").
set -e
export MSYS_NO_PATHCONV=1
SRC="$(cd "$(dirname "$0")/../.." && pwd)"
JDK="$1"; shift
case "$SRC" in
  *OneDrive*)
    DST="${FRC_MIRROR:-$LOCALAPPDATA/frc-build/FRC-CODE-MS}"
    mkdir -p "$DST"
    SRCW="$(cygpath -w "$SRC")"; DSTW="$(cygpath -w "$DST")"
    robocopy "$SRCW" "$DSTW" /MIR /XD build bin .gradle ctre_sim telemetria tools /XF "*.zip" /NFL /NDL /NJH /NJS /NP > /dev/null || true
    cd "$DST"
    ;;
  *)
    cd "$SRC"
    ;;
esac
export JAVA_HOME="$(cygpath -w "$JDK")"
./gradlew "$@" --console=plain -PteamNumber=10019
