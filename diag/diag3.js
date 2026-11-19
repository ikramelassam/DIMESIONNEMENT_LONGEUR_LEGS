const M=require('/home/claude/gr2_dim/sim/model.js');
const cfg=require('/home/claude/gr2_dim/audit/config.json'); cfg.gabarit=400;
const OUT={};
const NAMES=M.LEG_NAMES, base={L2:60.08,L3:71.92,alpha:10.55,beta:99.76};
for(const xi of [45,77.32]){
  const p=JSON.parse(JSON.stringify(cfg)); p.chassis.mount='dedans'; const ix=n=>p.servos.findIndex(s=>s.name===n);
  const r=M.evaluate(p,{...base,xi},'S2',{mode:'exact',combo:['S3306','S3306','S3306'].map(ix)});
  const ph=r.servo.phases[0], s=ph.best, Mc=r.servo.Mc; const L3=base.L3;
  console.log('\n===== xi',xi,'avant_RL, cellule retenue sway',r.sw[s],' Mc =',Mc.toFixed(3),'kg ; Mc·g =',(Mc*M.G).toFixed(2),'N');
  let kmax=[0,0,0],arg=['','',''];
  OUT[xi]={sway:r.sw[s],Mc,states:[],kmax:null,tau:null,model:ph.tau};
  r.detail[0].forEach((d,j)=>{const v=M.stateView(p,r,0,j,s);
    console.log(' état',d.label); const st={label:d.label,legs:[]}; OUT[xi].states.push(st);
    for(const l of v.order){const ps=v.legs[l]; const lam=v.lamLeg[l]; const be=ps.be; const k=[lam*(M.G*0+0.5/M.G)*ps.rh, lam*ps.D, lam*L3*Math.abs(Math.cos(be))];
      ['C1','C2','C3'].forEach((n,q)=>{if(k[q]>kmax[q]){kmax[q]=k[q];arg[q]=NAMES[l]+' / '+d.label;}});
      st.legs.push({leg:NAMES[l],lam,D:ps.D,beta:be*180/Math.PI,x3:L3*Math.abs(Math.cos(be)),k:k}); console.log('   ',NAMES[l],'λmax',lam.toFixed(2),'| D',ps.D.toFixed(1),'| α',ps.th2.toFixed(1),'β',(be*180/Math.PI).toFixed(1),'θ3',ps.th3.toFixed(1),'| L3|cosβ|',(L3*Math.abs(Math.cos(be))).toFixed(1),'| λ·D',k[1].toFixed(1),'| λ·L3|cosβ|',k[2].toFixed(1),'| λ·a/g·rh',k[0].toFixed(2));}
  });
  console.log(' => k max (mm) C1/C2/C3',kmax.map(v=>v.toFixed(2)).join(' / '),' | pire cas :',arg.join(' ; '));
  OUT[xi].kmax=kmax; OUT[xi].tau=kmax.map(k=>Mc*M.G*k/1000); OUT[xi].arg=arg;
  console.log(' => τ = Mc·g·k/1000 :',kmax.map(k=>(Mc*M.G*k/1000).toFixed(3)).join(' / '),' | modèle :',ph.tau.map(v=>v.toFixed(3)).join(' / '));
}

require('fs').writeFileSync('/home/claude/gr2_dim/diag/diag3.json',JSON.stringify(OUT));
