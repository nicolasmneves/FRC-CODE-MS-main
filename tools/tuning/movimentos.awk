# Lista, em janelas de 2 s, quais sensores dos modulos se moveram (graus para angulo, rotacoes para drive).
BEGIN { FS="," }
NR>1 && $2 ~ /Raw Absolute Encoder|Raw Angle Encoder|Raw Drive Encoder/ {
  split($2,p,"/"); m=p[5]
  k = ($2 ~ /Absolute/) ? "enc" : ($2 ~ /Angle/) ? "rot" : "drv"
  v = $3+0; if (k=="rot") v *= 360
  key = m "." k
  if (key in prev) { d = v - prev[key]; if (k=="enc") { if (d>180) d-=360; if (d<-180) d+=360 }; w = int($1/2); acc[w, key] += d; if (w>maxw) maxw=w; keys[key]=1 }
  prev[key] = v
}
END {
  for (w=0; w<=maxw; w++) { line=""; for (key in keys) { d=acc[w,key]; lim = (key ~ /drv/) ? 0.3 : 5; if (d>lim || d< -lim) line = line sprintf("  %s %+.1f", key, d) } if (line!="") printf "t=%3ds%s\n", w*2, line }
}
