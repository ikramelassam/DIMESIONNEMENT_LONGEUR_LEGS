const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76}; const out={};
for(const xi of [45,77.32]){
  const p=JSON.parse(JSON.stringify(cfg)); p.chassis.mount='dedans'; const ix=n=>p.servos.findIndex(s=>s.name===n);
  const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});
  const ph=r.servo.phases[0], s=ph.best; const j=1; const v=M.stateView(p,r,0,j,s);
  const rest=r.geo;
  out[xi]={sway:r.sw[s],order:v.order,feet:Object.fromEntries(Object.entries(v.legs).map(([l,ps])=>[l,ps.rel])),C1:rest.C1,P0:rest.P0,restFoot:rest.foot,Gc:v.Gc,hxy:v.hxy,swing:v.swing,N:p.chassis.N_ch,M:p.chassis.M_ch,
   // pied au repos par rapport au corps décalé (pour montrer la déformation)
  };
}
require('fs').writeFileSync('/home/claude/gr2_dim/diag/topview.json',JSON.stringify(out)); console.log('ok');
