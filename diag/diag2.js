const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
function mk(mod){const p=JSON.parse(JSON.stringify(cfg)); p.chassis.mount='dedans'; if(mod)mod(p); return p;}
function ev(xi,mod,sc='S2'){const p=mk(mod);const ix=n=>p.servos.findIndex(s=>s.name===n);return {p,r:M.evaluate(p,{...base,xi},sc,{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)})};}
// 1) cartes de la phase avant_RL (121 cellules) : x en ligne (avant en haut), y en colonne
for(const xi of [45,77.32]){
  const {p,r}=ev(xi); const NS=r.sw.length,n=11;
  console.log('\n=== carte avant_RL xi',xi,' (lignes : décalage x de +60 en haut à -60 ; colonnes : y de -60 à +60)  chiffre = NESM mm (cellule tenable), x = géométrie KO, s = stable KO / NESM<0');
  const lin=[...new Set(r.sw.map(q=>q[0]))];
  for(let i=n-1;i>=0;i--){let line=String(lin[i].toFixed(0)).padStart(4)+' |';
    for(let j=0;j<n;j++){const s=i*n+j; const geo=r.geoOk[0][s], nes=r.nesm[0][s], ssm=r.ssmc[0][s];
      line+= !geo?'   x ' : (nes>=0&&ssm>=0? String(nes.toFixed(1)).padStart(4)+' ' : '   s ');}
    console.log(line);}
  // causes geometriques
  const bits=new Array(7).fill(0),only=new Array(7).fill(0); for(let s=0;s<NS;s++){const w=r.why[0][s];for(let b=0;b<7;b++){if(w&(1<<b))bits[b]++;if(w===(1<<b))only[b]++;}}
  console.log(' causes (cellules touchées / seule cause):',M.WHY.map((nm,i)=>bits[i]?nm+' '+bits[i]+'/'+only[i]:'').filter(Boolean).join(' ; '));
}
// 2) attribution : on relâche une hypothèse à la fois ; f2 global, phases ok, NESM max phase avant_RL
const relax={
 'référence (S2)':null,
 'marge pied/châssis 10→0':p=>{p.margins.chassis_clearance=0;},
 'débord C2 illimité':p=>{p.chassis.overhang_C2=1e4;},
 'distance min pieds 30→0':p=>{p.margins.feet_clearance=0;},
 'marge de portée 5→0':p=>{p.margins.reach_margin=0;},
 'θ3 ∈ [0,180]':p=>{p.margins.theta3_min=-180;p.margins.theta3_max=360;},
 'erreur pied 5→0':p=>{p.margins.eps_foot=0;},
 'accélération 0.5→0':p=>{p.task.accel=0;},
 'incertitude CdG ±10→0':p=>{p.body.eps_cdg_xy=0;p.body.eps_cdg_z=0;},
 'balancement max 60→100 (grille 17)':p=>{p.task.sway_max=100;p.task.sway_n=17;},
};
const ATT=[];console.log('\n=== attribution (S2, servos S3306×3, mêmes L2,L3,α,β)');
console.log('relaxation'.padEnd(36),'| ξ=45 : phases  f2   avant_RL | ξ=77 : phases  f2   avant_RL');
for(const [k,mod] of Object.entries(relax)){
  let line=k.padEnd(36)+'|'; const rec={name:k};
  for(const xi of [45,77.32]){const {r}=ev(xi,mod);const s=r.servo;const a=s.phases[0];
    rec[xi]={ph:s.phases.filter(q=>q.ok).length,f2:Number.isFinite(s.f2)?s.f2:null,a:a.ok?a.nesm:null};
    line+=` ${String(s.phases.filter(q=>q.ok).length).padStart(2)}/14  ${Number.isFinite(s.f2)?s.f2.toFixed(1).padStart(5):'  n/a'}  ${a.ok?a.nesm.toFixed(1).padStart(5):'  KO '}    |`;}
  console.log(line); ATT.push(rec);
}
require('fs').writeFileSync('/home/claude/gr2_dim/diag/attr.json',JSON.stringify(ATT));
// 3) S1 (CdG nominal, boîte nulle)
for(const xi of [45,77.32]){const {r}=ev(xi,null,'S1');const s=r.servo;console.log('S1 nominal xi',xi,'phases',s.phases.filter(q=>q.ok).length,'f2',Number.isFinite(s.f2)?s.f2.toFixed(1):'-');}
