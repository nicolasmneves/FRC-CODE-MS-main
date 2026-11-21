#!/usr/bin/env bash
# Atalho para o cliente NT4: ./nt.sh dump 3 /SmartDashboard/swerve
export MSYS_NO_PATHCONV=1
JAVA="${JAVA_NT:-/c/Users/Public/wpilib/2027_alpha7/jdk/bin/java}"
exec "$JAVA" "$(cygpath -w "$(dirname "$0")")\Nt.java" "${RIO:-roborio-10019-frc.local}" "$@"
