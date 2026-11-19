import json, numpy as np, pandas as pd, os, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Polygon
D="/home/claude/gr2_dim"; OUT=sys.argv[1]; os.makedirs(OUT,exist_ok=True)
plt.rcParams.update({"font.size":9,"axes.spines.top":False,"axes.spines.right":False,"axes.grid":True,"grid.alpha":.25,"figure.dpi":150})
C={"45":"#c4600f","77":"#0b6b7c","gray":"#8a949b"}
# ---- fig 1 : balayage xi
sw=json.load(open(f"{D}/diag/sweep_xi.json")); xi=[r["xi"] for r in sw]
fig,ax=plt.subplots(1,3,figsize=(11.5,3.4))
ax[0].plot(xi,[r["S1_sss"]["f2"] for r in sw],"-o",ms=3,color="#2f6fb0",label="CdG nominal (S1)")
ax[0].plot(xi,[r["S2_sss"]["f2"] for r in sw],"-o",ms=3,color="#b3261e",label="CdG pire cas (S2)")
ax[0].set_xlabel("ξ au repos (°)"); ax[0].set_ylabel("$f_2$ = NESM min. sur 14 phases (mm)"); ax[0].legend(frameon=True,framealpha=0.9,fontsize=7.5,loc="upper center",bbox_to_anchor=(0.5,-0.18),ncol=2); ax[0].set_title("(a) marge de stabilité",fontsize=9)
ax[1].plot(xi,[r["S2_geo_avantRL"] for r in sw],"-o",ms=3,color="#555")
ax[1].set_xlabel("ξ au repos (°)"); ax[1].set_ylabel("positions du corps atteignables\n(sur 121) en phase avant_RL"); ax[1].set_title("(b) place pour balancer le corps",fontsize=9)
ax[2].plot(xi,[r["S2_sss"]["t"][2] for r in sw],"-o",ms=3,color="#6a4fa3",label="C3 (genou)")
ax[2].plot(xi,[r["S2_sss"]["t"][1] for r in sw],"-o",ms=3,color="#0b7a8c",label="C2 (hanche)")
ax[2].axhline(0.58,color=C["gray"],ls="--",lw=1); ax[2].text(1,0.6,"limite utilisable S-U400 (0,58)",fontsize=7,color=C["gray"])
ax[2].set_xlabel("ξ au repos (°)"); ax[2].set_ylabel("couple requis minimal (N·m)"); ax[2].legend(frameon=True,framealpha=0.9,fontsize=8,loc="upper center",bbox_to_anchor=(0.5,-0.18),ncol=2); ax[2].set_title("(c) couple demandé",fontsize=9)
for a in ax:
    for v,c in ((45,C["45"]),(77.32,C["77"])): a.axvline(v,color=c,lw=1,alpha=.7)
fig.tight_layout(); fig.savefig(f"{OUT}/xi_sweep.png"); plt.close(fig)
# ---- fig 2 : cartes de balancement
d=json.load(open(f"{D}/diag/diag1.json"))
fig,ax=plt.subplots(1,2,figsize=(9.6,4.2))
for a,k,t in zip(ax,("45","77.32"),("ξ = 45°","ξ = 77,3°")):
    rows=d[k]["rows"]; n=11; Z=np.full((n,n),np.nan); kind=np.zeros((n,n),int)
    lin=np.linspace(-60,60,n)
    for r in rows:
        i,j=divmod(r["s"],n)
        if not r["geoOk"]: kind[i,j]=0
        elif r["nes"]>=0 and r["ssm"]>=0: kind[i,j]=2; Z[i,j]=r["nes"]
        else: kind[i,j]=1
    cmap=plt.get_cmap("YlGn"); 
    for i in range(n):
        for j in range(n):
            x,y=lin[j],lin[i]
            col="#d9dde0" if kind[i,j]==0 else ("#f3c9a8" if kind[i,j]==1 else cmap(0.25+0.75*min(Z[i,j],7)/7))
            a.add_patch(Rectangle((x-6,y-6),12,12,color=col,lw=0))
            if kind[i,j]==2: a.text(x,y,f"{Z[i,j]:.1f}",ha="center",va="center",fontsize=6.5)
    bs=d[k]["sw"]; a.add_patch(Rectangle((bs[1]-6,bs[0]-6),12,12,fill=False,ec="#b3261e",lw=1.6))
    a.set_xlim(-66,66); a.set_ylim(-66,66); a.set_aspect("equal"); a.set_xlabel("décalage du corps en y (mm, + = gauche)"); a.set_ylabel("décalage en x (mm, + = avant)"); a.set_title(f"{t} : {sum(1 for r in rows if r['geoOk'])}/121 positions atteignables",fontsize=9); a.grid(False)
fig.text(0.5,0.005,"gris : une patte ne peut pas atteindre · orange : atteignable mais instable (NESM < 0) · vert : tenable, chiffre = NESM (mm) · cadre rouge : position retenue",ha="center",fontsize=7.5)
fig.tight_layout(rect=(0,0.03,1,1)); fig.savefig(f"{OUT}/sway_maps.png"); plt.close(fig)
# ---- fig 3 : vue de dessus du tripode
tv=json.load(open(f"{D}/diag/topview.json"))
fig,ax=plt.subplots(1,2,figsize=(9.6,4.6))
NM=["FL","FR","RL","RR"]
for a,k,t in zip(ax,("45","77.32"),("ξ = 45°","ξ = 77,3°")):
    v=tv[k]; N,Mm=v["N"],v["M"]
    X=lambda y:-y; Y=lambda x:x
    a.add_patch(Rectangle((-Mm/2,-N/2),Mm,N,color="#c9d2d8",alpha=.7,lw=0)); a.text(0,-N/2+10,"châssis 200×200",ha="center",fontsize=7,color="#555")
    ground=[l for l in v["order"] if l!=v["swing"]]
    pts=[v["feet"][str(l)] for l in ground]
    # ordre cyclique deja donne par 'order'
    a.add_patch(Polygon([[X(p[1]),Y(p[0])] for p in pts],closed=True,color="#0b6b7c",alpha=.13,ec="#0b6b7c"))
    for l in range(4):
        c1=v["C1"][l]; p0=v["P0"][l]; f=v["feet"][str(l)]
        col="#999" if l==v["swing"] else "#0b7a8c"
        a.plot([X(c1[1]),X(p0[1]),X(f[1])],[Y(c1[0]),Y(p0[0]),Y(f[0])],color=col,lw=2,ls="--" if l==v["swing"] else "-")
        a.scatter([X(f[1])],[Y(f[0])],s=34,color="#222" if l!=v["swing"] else "white",ec="#222",zorder=3)
        a.text(X(f[1])+(8 if f[1]<0 else -8),Y(f[0])+(8 if f[0]>0 else -14),NM[l],fontsize=8,ha="left" if f[1]<0 else "right")
    gx,gy=v["Gc"]; hx,hy=v["hxy"]
    a.add_patch(Rectangle((X(gy)-hy,Y(gx)-hx),2*hy,2*hx,fill=False,ec="#b3261e",lw=1.4)); a.scatter([X(gy)],[Y(gx)],color="#b3261e",s=14,zorder=4)
    a.set_xlim(-190,190); a.set_ylim(-190,190); a.set_aspect("equal"); a.set_title(f"{t} : phase avant_RL, patte RL en vol",fontsize=9); a.grid(False); a.set_xticks([]); a.set_yticks([])
fig.text(0.5,0.005,"rouge : boîte du CdG au pire cas (S2) à la position retenue · bleu : polygone d'appui à 3 pieds · ↑ avant",ha="center",fontsize=7.5)
fig.tight_layout(rect=(0,0.03,1,1)); fig.savefig(f"{OUT}/tripod.png"); plt.close(fig)
# ---- fig 4 : fronts A vs B (+ D/E si présents)
def load(n):
    p=f"{D}/results_study/{n}"
    return (pd.read_csv(f"{p}/admissibles.csv"),pd.read_csv(f"{p}/pareto.csv")) if os.path.exists(f"{p}/pareto.csv") else (None,None)
sets=[("A_xi_libre","ξ libre","#0b6b7c"),("B_xi_45","ξ = 45° imposé","#c4600f")]
fig,ax=plt.subplots(1,2,figsize=(9.8,3.8))
for n,lab,c in sets:
    ad,fr=load(n)
    for a,(x,xl) in zip(ax,(("f1_masse_pattes","$f_1$ masse des 4 pattes (kg)"),("f3_couple_C2","$f_3$ couple $C_2$ requis (N·m)"))):
        a.scatter(ad[x],ad.f2_nesm,s=3,color=c,alpha=.18,lw=0)
        fr2=fr.sort_values(x); a.plot(fr2[x],fr2.f2_nesm,"o",ms=3.2,color=c,label=lab+" (front de Pareto)")
        a.set_xlabel(xl); a.set_ylabel("$f_2$ NESM (mm)")
ax[0].legend(frameon=False,fontsize=7.5)
fig.tight_layout(); fig.savefig(f"{OUT}/pareto_AB.png"); plt.close(fig)
# ---- fig 5 : couples requis vs limites des servos
sw_=json.load(open(f"{D}/diag/sweep_xi.json")); r45=[r for r in sw_ if r["xi"]==45][0]["S2_sss"]["t"]; r77=[r for r in sw_ if r["xi"]==77.5][0]["S2_sss"]["t"]
servos=[("SG90",.147,.014),("S-U400",.696,.039),("S3306",2.354,.126),("AX-12A",1.5,.0546),("MX-28T",2.5,.072),("MX-64AT",6.0,.126),("MX-106T",8.4,.153)]
fig,ax=plt.subplots(figsize=(8.6,3.8)); ax.set_yscale("log")
for i,(n,cs,m) in enumerate(servos):
    ax.bar(i,cs/1.2,color="#c9d2d8" if n in("SG90","S-U400","S3306") else "#e6e9ec",ec="#8a949b",width=.6)
    ax.text(i,cs/1.2*1.08,f"{cs/1.2:.2f}\n{m*1000:.0f} g",ha="center",fontsize=7)
ax.set_xticks(range(len(servos))); ax.set_xticklabels([s[0] for s in servos])
for lab,t,c,ls in (("ξ = 77° : C3",r77[2],"#0b6b7c","-"),("ξ = 45° : C3",r45[2],"#c4600f","-"),("ξ = 77° : C2",r77[1],"#0b6b7c","--"),("ξ = 45° : C2",r45[1],"#c4600f","--"),("C1 (77° et 45°)",r45[0],"#6a4fa3",":")):
    ax.axhline(t,color=c,ls=ls,lw=1.3); ax.text(6.55,t,f"{lab} = {t:.2f}",fontsize=7,va="center",color=c)
ax.set_xlim(-.5,8.9); ax.set_ylim(.07,12); ax.set_ylabel("couple (N·m)"); ax.set_title("Barres : couple utilisable de chaque servo (blocage ÷ 1,2) · lignes : couple requis (patte S2 'dedans')",fontsize=8.5)
fig.tight_layout(); fig.savefig(f"{OUT}/servo_need.png"); plt.close(fig)
print("figs ok")
