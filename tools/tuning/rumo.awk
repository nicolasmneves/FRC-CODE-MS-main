# Resume cada vez que o analogico direito foi segurado: alvo, erro ao longo do tempo, overshoot e erro final.
BEGIN { FS="," }
NR>1 {
  n=$2; sub(/.*\//,"",n)
  if (n=="Apontando") { ap = ($3=="true") }
  else if (n=="AlvoGraus") alvo=$3+0
  else if (n=="ErroGraus") {
    e=$3+0
    if (ap && !em) { em=1; t0=$1; e0=e; s=(e>=0)?1:-1; ov=0; tset=-1 }
    if (ap && em) { if (e*s < 0 && -e*s > ov) ov = -e*s; if ((e<0?-e:e) > 2) tset=$1-t0; efim=e; tfim=$1-t0 }
    if (!ap && em) { em=0; printf "alvo %7.1f°  erro inicial %7.1f°  overshoot %5.1f°  entra em +-2° apos %5.2f s  erro ao soltar %6.2f° (segurou %.1f s)\n", alvo, e0, ov, tset, efim, tfim }
  }
}
