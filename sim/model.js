/* Gr2 - modele de dimensionnement des pattes, portage JavaScript EXACT de gr2/
 * (params.py, kinematics.py, statics.py, gait.py, evaluate.py).
 * Memes equations, memes conventions, memes ordres de boucles : verifie contre
 * le programme Python par sim/validate.js (ecarts ~1e-9).
 * Unites : mm, kg, N.m, degres. Repere corps : x avant, y gauche, z haut.
 */
(function (root) {
  "use strict";
  const G = 9.81;
  const LEG_NAMES = ["FL", "FR", "RL", "RR"];
  const LEG_SIGNS = [[+1, +1], [+1, -1], [-1, +1], [-1, -1]];
  const CYCLIC = [0, 2, 3, 1];
  const FL = 0, FR = 1, RL = 2, RR = 3;
  const MIRROR = { 0: 2, 2: 0, 1: 3, 3: 1 };
  const rad = (d) => (d * Math.PI) / 180, deg = (r) => (r * 180) / Math.PI;
  const clip = (v, a, b) => Math.min(b, Math.max(a, v));

  // ------------------------------------------------------------- servos
  function servoList(p) {
    const fr = p.frac_couple ?? 1.0;
    return p.servos.map((s) => ({ ...s, frac: fr, C_cont: fr * s.C_stall }));
  }
  function combos(p) {
    const S = servoList(p), out = [];
    for (let a = 0; a < S.length; a++) for (let b = 0; b < S.length; b++) for (let c = 0; c < S.length; c++) {
      const cc = [a, b, c]; let ok = true;
      for (const i of new Set(cc)) {
        const st = S[i].stock; const n = cc.filter((v) => v === i).length;
        if (st !== null && st !== undefined && 4 * n > st) ok = false;
      }
      if (ok) out.push(cc);
    }
    return out;
  }

  // ------------------------------------------------------------- masses
  function legMass(p, L2, L3, m1, m2, m3) {
    const lg = p.leg, lam = lg.seg_lin_mass / 1000;
    return m1 + m2 + m3 + lam * (lg.L1 + L2 + L3) + 3 * lg.bracket_mass + lg.foot_mass;
  }
  function armMass(p) { const a = p.body.arm_model; return a.m_base + a.m_elbow + a.m_tip; }
  function armPoints(p, posture) {
    const b = p.body, a = b.arm_model, ps = a.postures[posture];
    const f1 = rad(ps.phi[0]), f2 = rad(ps.phi[0] + ps.phi[1]);
    const S = [p.chassis.N_ch / 2 - a.mount_from_front, a.mount_y, a.mount_z_rel];
    const E = [S[0] + a.a1 * Math.cos(f1), S[1], S[2] + a.a1 * Math.sin(f1)];
    const T = [E[0] + a.a2 * Math.cos(f2), E[1], E[2] + a.a2 * Math.sin(f2)];
    const pts = [[a.m_base, S, "epaule"], [a.m_elbow, E, "coude"], [a.m_tip, T, "pince"]];
    if (ps.obj) pts.push([b.object_mass, T.slice(), "objet"]);
    return pts;
  }
  function bodyCdg(p, bat, posture) {
    const b = p.body, ch = b.chassis;
    const items = [[ch.m, [ch.x, ch.y, ch.z_rel]], [b.battery.m, bat.slice()]].concat(armPoints(p, posture));
    let m = 0; const g = [0, 0, 0];
    for (const [mi, pi] of items) { m += mi; for (let k = 0; k < 3; k++) g[k] += mi * pi[k]; }
    return [m, g.map((v) => v / m)];
  }
  function bodyNominal(p, posture) {
    const bt = p.body.battery; return bodyCdg(p, [bt.x, bt.y, bt.z_rel], posture);
  }
  function bodyBox(p, postures) {
    const z = p.body.battery_zone; const pts = [], masses = [];
    for (const post of postures || Object.keys(p.body.arm_model.postures))
      for (const bx of z.x) for (const by of z.y) for (const bz of z.z_rel) {
        const [m, gp] = bodyCdg(p, [bx, by, bz], post); pts.push(gp); masses.push(m);
      }
    const lo = [0, 1, 2].map((k) => Math.min(...pts.map((q) => q[k])));
    const hi = [0, 1, 2].map((k) => Math.max(...pts.map((q) => q[k])));
    return [Math.max(...masses), lo.map((v, k) => (v + hi[k]) / 2), lo.map((v, k) => (hi[k] - v) / 2)];
  }

  // ----------------------------------------------------------- cinematique
  function ik(L2, L3, D, Hz) {
    const d = Math.hypot(D, Hz);
    const reach = d <= L2 + L3 && d >= Math.abs(L2 - L3) && d > 1e-9;
    const cpsi = (L2 * L2 + d * d - L3 * L3) / (2 * L2 * Math.max(d, 1e-9));
    const psi = Math.acos(clip(cpsi, -1, 1));
    const al = psi - Math.atan2(Hz, D);
    const be = Math.atan2(Hz + L2 * Math.sin(al), D - L2 * Math.cos(al));
    return { al, be, d, reach };
  }

  // ------------------------------------------------------------- geometrie
  function geometry(p, x) {
    const L1 = p.leg.L1, ch = p.chassis;
    const [L2, L3] = [x.L2, x.L3];
    const a = rad(x.alpha), b = rad(x.beta), xi = rad(x.xi);
    const D0 = L2 * Math.cos(a) + L3 * Math.cos(b);
    const H2 = L3 * Math.sin(b) - L2 * Math.sin(a);
    const h = H2 - ch.e;
    const u = [], C1 = [], foot = [], P0 = [];
    for (let i = 0; i < 4; i++) {
      const s = LEG_SIGNS[i];
      u.push([s[0] * Math.sin(xi), s[1] * Math.cos(xi)]);
      P0.push([s[0] * (ch.N_ch / 2 - ch.c), s[1] * (ch.M_ch / 2 - ch.c)]);
      if (ch.mount === "dehors") {
        C1.push(P0[i].slice()); foot.push([P0[i][0] + (L1 + D0) * u[i][0], P0[i][1] + (L1 + D0) * u[i][1]]);
      } else {
        C1.push([P0[i][0] - L1 * u[i][0], P0[i][1] - L1 * u[i][1]]);
        foot.push([P0[i][0] + D0 * u[i][0], P0[i][1] + D0 * u[i][1]]);
      }
    }
    return { L2, L3, a, b, xi, D0, H2, h, u, C1, foot, P0 };
  }

  function prefilter(p, geo) {
    const tk = p.task, mg = p.margins, ch = p.chassis, lg = p.leg;
    const v = {};
    const hmin = tk.obstacle + tk.h_margin;
    v.G5_garde_sol = (hmin - geo.h) / hmin;
    const th3 = deg(geo.a + geo.b);
    v.G3_theta3_repos = Math.max(mg.theta3_min - th3, th3 - mg.theta3_max) / 90;
    const gab = p.gabarit ?? 400;
    const ex = 2 * Math.max(...geo.foot.map((f) => Math.abs(f[0]))) + lg.foot_diam;
    const ey = 2 * Math.max(...geo.foot.map((f) => Math.abs(f[1]))) + lg.foot_diam;
    v.G6_gabarit = Math.max(ex, ey) / gab - 1;
    // EMPREINTE IMPOSEE (ajout 17/11) : gabarit_min = 0 -> contrainte inactive
    const gabMin = p.gabarit_min ?? 0;
    if (gabMin > 0) v.G6b_empreinte_min = (gabMin - Math.max(ex, ey)) / gabMin;
    const sp = mg.servo_spacing ?? 40;
    const dx = geo.C1[0][0] - geo.C1[2][0], dy = geo.C1[0][1] - geo.C1[1][1];
    v.G7_axes_C1 = (sp - Math.min(dx, dy)) / sp;
    v.G10_chassis = Math.max(ch.N_ch, ch.M_ch) / gab - 1;
    const ok = Object.values(v).every((q) => q <= 0);
    return { ok, viol: v, ext: [ex, ey], th3 };
  }

  // ----------------------------------------------------------------- allure
  function buildPhases(R, liftZ, obstacle, yawStep) {
    const order = [RL, FL, RR, FR], phases = [];
    function cycle(name, mirror, onObst) {
      order.forEach((k, ki) => {
        const at = (j) => order[((j % 4) + 4) % 4];
        const p1 = at(ki - 1), p2 = at(ki - 2), p3 = at(ki - 3);
        let offs = new Map([[k, -R / 2], [p1, R / 4], [p2, 0.0], [p3, -R / 4]]);
        let kk = k;
        if (mirror) { const m = new Map(); for (const [l, o] of offs) m.set(MIRROR[l], -o); offs = m; kk = MIRROR[k]; }
        const z = (l) => (onObst && (l === FL || l === FR) ? obstacle : 0.0);
        const nm = `${name}_${LEG_NAMES[kk]}`;
        const ph = { name: nm, states: [] };
        ph.states.push({ ground: [...offs].map(([l, o]) => [l, o, z(l)]), swing: null, yaw: 0, cdg: "walk", label: "pré-levée (4 pieds)" });
        ph.states.push({ ground: [...offs].filter(([l]) => l !== kk).map(([l, o]) => [l, o, z(l)]), swing: [kk, 0.0, liftZ], yaw: 0, cdg: "walk", label: "vol (3 pieds)" });
        const post = new Map(offs); post.set(kk, -post.get(kk));
        ph.states.push({ ground: [...post].map(([l, o]) => [l, o, z(l)]), swing: null, yaw: 0, cdg: "walk", label: "post-pose (4 pieds)" });
        phases.push(ph);
      });
    }
    cycle("avant", false, false); cycle("arriere", true, false); cycle("obstacle", false, true);
    const rot = { name: "rotation", states: [] };
    for (const s of [-1, 1]) rot.states.push({ ground: [0, 1, 2, 3].map((l) => [l, 0, 0]), swing: null, yaw: (s * yawStep) / 2, cdg: "walk", label: `lacet ${s > 0 ? "+" : ""}${(s * yawStep) / 2} deg` });
    phases.push(rot);
    phases.push({ name: "saisie", states: [{ ground: [0, 1, 2, 3].map((l) => [l, 0, 0]), swing: null, yaw: 0, cdg: "manip", label: "bras tendu" }] });
    return phases;
  }

  // ---------------------------------------------------------------- statique
  function barycentric(pt, a, b, c) {
    const [x, y] = pt, [x1, y1] = a, [x2, y2] = b, [x3, y3] = c;
    let det = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);
    if (Math.abs(det) < 1e-9) det = 1e-9;
    const g1 = [(y2 - y3) / det, (x3 - x2) / det], g2 = [(y3 - y1) / det, (x1 - x3) / det];
    const l1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / det;
    const l2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / det;
    return { lam: [l1, l2, 1 - l1 - l2], grad: [g1, g2, [-g1[0] - g2[0], -g1[1] - g2[1]]] };
  }
  const boxMax = (f, g, h) => f + Math.abs(g[0]) * h[0] + Math.abs(g[1]) * h[1];
  const boxMin = (f, g, h) => f - Math.abs(g[0]) * h[0] - Math.abs(g[1]) * h[1];
  function edgeInward(pa, pb, cen) {
    const e = [pb[0] - pa[0], pb[1] - pa[1]];
    let n = [-e[1], e[0]]; const nn = Math.max(Math.hypot(n[0], n[1]), 1e-12); n = [n[0] / nn, n[1] / nn];
    let s = Math.sign(n[0] * (cen[0] - pa[0]) + n[1] * (cen[1] - pa[1])); if (s === 0) s = 1;
    return [n[0] * s, n[1] * s];
  }
  function polygonMargins(feet, Gc, h, zG) {
    const n = feet.length; const cen = [0, 0];
    for (const f of feet) { cen[0] += f[0] / n; cen[1] += f[1] / n; }
    let ssm = Infinity, nesm = Infinity; const edges = [];
    for (let k = 0; k < n; k++) {
      const pa = feet[k], pb = feet[(k + 1) % n], nr = edgeInward(pa, pb, cen);
      const dc = nr[0] * (Gc[0] - pa[0]) + nr[1] * (Gc[1] - pa[1]);
      const dmin = boxMin(dc, nr, h);
      const ne = dmin >= 0 ? Math.hypot(dmin, zG) - zG : dmin;
      ssm = Math.min(ssm, dmin); nesm = Math.min(nesm, ne); edges.push(dmin);
    }
    return { ssm, nesm, edges };
  }
  function nesm3d(feet3, G3) {
    const n = feet3.length; const cen = [0, 0];
    for (const f of feet3) { cen[0] += f[0] / n; cen[1] += f[1] / n; }
    let out = Infinity;
    for (let k = 0; k < n; k++) {
      const P1 = feet3[k], P2 = feet3[(k + 1) % n];
      const nr = edgeInward(P1, P2, cen);
      const dh = nr[0] * (G3[0] - P1[0]) + nr[1] * (G3[1] - P1[1]);
      let a = [P2[0] - P1[0], P2[1] - P1[1], P2[2] - P1[2]]; const an = Math.hypot(...a); a = a.map((v) => v / an);
      const t = (G3[0] - P1[0]) * a[0] + (G3[1] - P1[1]) * a[1] + (G3[2] - P1[2]) * a[2];
      const Q = [P1[0] + t * a[0], P1[1] + t * a[1], P1[2] + t * a[2]];
      const rho = Math.hypot(G3[0] - Q[0], G3[1] - Q[1], G3[2] - Q[2]);
      const top = Q[2] + rho * Math.sqrt(clip(1 - a[2] * a[2], 0, 1));
      out = Math.min(out, dh >= 0 ? top - G3[2] : dh);
    }
    return out;
  }
  function fourFeetMaxLambda(f4, Gc, h) {
    const out = [0, 0, 0, 0];
    for (let omit = 0; omit < 4; omit++) {
      const idx = [0, 1, 2, 3].filter((i) => i !== omit);
      const { lam, grad } = barycentric(Gc, f4[idx[0]], f4[idx[1]], f4[idx[2]]);
      const lmax = lam.map((l, j) => boxMax(l, grad[j], h));
      const plaus = lmax.every((v) => v >= 0);
      idx.forEach((leg, j) => { const val = plaus ? clip(lmax[j], 0, 1) : 0; out[leg] = Math.max(out[leg], val); });
    }
    return out;
  }
  function segSegDist(p1, p2, q1, q2) {
    const ptSeg = (p, a, b) => {
      const ab = [b[0] - a[0], b[1] - a[1]];
      const t = clip(((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / Math.max(ab[0] * ab[0] + ab[1] * ab[1], 1e-12), 0, 1);
      return Math.hypot(p[0] - (a[0] + t * ab[0]), p[1] - (a[1] + t * ab[1]));
    };
    const cross = (u, v) => u[0] * v[1] - u[1] * v[0];
    const d = Math.min(ptSeg(p1, q1, q2), ptSeg(p2, q1, q2), ptSeg(q1, p1, p2), ptSeg(q2, p1, p2));
    const r = [p2[0] - p1[0], p2[1] - p1[1]], s = [q2[0] - q1[0], q2[1] - q1[1]];
    const den = cross(r, s); const dens = Math.abs(den) < 1e-12 ? 1e-12 : den;
    const qp = [q1[0] - p1[0], q1[1] - p1[1]];
    const t = cross(qp, s) / dens, uu = cross(qp, r) / dens;
    const inter = Math.abs(den) > 1e-12 && t >= 0 && t <= 1 && uu >= 0 && uu <= 1;
    return inter ? 0 : d;
  }

  // ------------------------------------------------------------- une patte
  function legPose(p, geo, i, off, zf, yaw, sw) {
    const L1 = p.leg.L1, mg = p.margins;
    let rel = [geo.foot[i][0] + off - sw[0], geo.foot[i][1] - sw[1]];
    if (yaw !== 0) {
      const c = Math.cos(rad(-yaw)), s = Math.sin(rad(-yaw));
      rel = [c * rel[0] - s * rel[1], s * rel[0] + c * rel[1]];
    }
    const v = [rel[0] - geo.C1[i][0], rel[1] - geo.C1[i][1]];
    const rh = Math.hypot(v[0], v[1]);
    const dirv = [v[0] / Math.max(rh, 1e-9), v[1] / Math.max(rh, 1e-9)];
    const D = rh - L1, Hz = geo.H2 - zf;
    const { al, be, d, reach: r0 } = ik(geo.L2, geo.L3, D, Hz);
    const m = mg.reach_margin;
    const reachMax = d <= geo.L2 + geo.L3 - m, reachMin = d >= Math.abs(geo.L2 - geo.L3) + m;
    const reach = r0 && reachMax && reachMin && D > 0;
    const th3 = deg(al + be);
    const u0 = geo.u[i];
    const th1 = deg(Math.atan2(u0[0] * dirv[1] - u0[1] * dirv[0], u0[0] * dirv[0] + u0[1] * dirv[1]));
    const C2 = [geo.C1[i][0] + L1 * dirv[0], geo.C1[i][1] + L1 * dirv[1]];
    const kx = [C2[0] + geo.L2 * Math.cos(al) * dirv[0], C2[1] + geo.L2 * Math.cos(al) * dirv[1]];
    const z2 = geo.H2;
    return {
      rel, D, rh, al, be, d, reach, reachMax: r0 && reachMax, reachMin: reachMin, Dpos: D > 0, th1, th2: deg(al), th3, dirv,
      C1: [geo.C1[i][0], geo.C1[i][1], z2], C2: [C2[0], C2[1], z2], knee: [kx[0], kx[1], z2 + geo.L2 * Math.sin(al)],
      foot: [rel[0], rel[1], zf], L2: geo.L2, L3: geo.L3, zf,
    };
  }
  // moment des pieces d'une patte (kg.mm). off = decalages des CdG des servos
  // le long du segment sortant (0 = sur l'axe, comme le programme Python).
  function legMoment(p, ps, m1, m2, m3, off) {
    const lg = p.leg, lam = lg.seg_lin_mass / 1000, br = lg.bracket_mass, L1 = lg.L1;
    const o = off || [0, 0, 0];
    const along = (A, B, dist) => { const L = Math.hypot(B[0] - A[0], B[1] - A[1], B[2] - A[2]) || 1; return A.map((v, k) => v + ((B[k] - A[k]) * dist) / L); };
    const S1 = o[0] ? along(ps.C1, ps.C2, o[0]) : ps.C1, S2 = o[1] ? along(ps.C2, ps.knee, o[1]) : ps.C2, S3 = o[2] ? along(ps.knee, ps.foot, o[2]) : ps.knee;
    const mom = [0, 0, 0];
    const add = (m, P) => { for (let k = 0; k < 3; k++) mom[k] += m * P[k]; };
    const mid = (A, B) => A.map((v, k) => 0.5 * (v + B[k]));
    add(m1, S1); add(lam * L1 + br, mid(ps.C1, ps.C2));
    add(m2, S2); add(br, ps.C2); add(lam * ps.L2, mid(ps.C2, ps.knee));
    add(m3, S3); add(br, ps.knee); add(lam * ps.L3, mid(ps.knee, ps.foot));
    add(lg.foot_mass, ps.foot);
    return mom;
  }

  // ------------------------------------------------------------- evaluation
  /* opts.mode : "programme" (identique a gr2/evaluate.py : servo de reference
   *   + borne eps_legs) ou "exact" (CdG des pattes avec les servos choisis,
   *   eps_legs = 0). opts.combo : indices [C1, C2, C3] dans p.servos.
   *   opts.servo_offsets : decalages des CdG des servos (mm), mode exact. */
  function evaluate(p, x, scenario, opts) {
    opts = opts || {};
    const mode = opts.mode || "programme";
    const S = servoList(p);
    const combo = opts.combo;
    const tk = p.task, mg = p.margins, ch = p.chassis, lg = p.leg, b = p.body;
    const geo = geometry(p, x);
    const pre = prefilter(p, geo);
    const phases = buildPhases(tk.stroke, tk.obstacle + tk.dh_lift, tk.obstacle, tk.yaw_step);
    const n = tk.sway_n, smax = tk.sway_max;
    const lin = Array.from({ length: n }, (_, i) => (n === 1 ? -smax : -smax + (2 * smax * i) / (n - 1)));
    const sw = []; for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) sw.push([lin[i], lin[j]]);
    const NS = sw.length;
    // corps
    let body, eps;
    if (scenario === "S1") {
      const [mw, gw] = bodyNominal(p, b.walk_posture), [mm, gm] = bodyNominal(p, b.manip_posture);
      body = { walk: [mw, gw, [0, 0, 0]], manip: [mm, gm, [0, 0, 0]] };
      const es = b.eps_s1 ?? 0; eps = [es, es, es];
    } else {
      body = { walk: bodyBox(p, b.walk_postures_S2), manip: bodyBox(p, b.manip_postures_S2) };
      eps = [b.eps_cdg_xy, b.eps_cdg_xy, b.eps_cdg_z];
    }
    const mBodyMax = Math.max(body.walk[0], body.manip[0]);
    // masses de patte pour le CdG
    const ref = S.find((s) => s.name === p.reference_servo);
    let mCdg, dm, Mmin;
    const masses = S.map((s) => s.mass);
    if (mode === "exact") {
      mCdg = combo.map((i) => S[i].mass); dm = 0;
    } else {
      mCdg = [ref.mass, ref.mass, ref.mass];
      dm = Math.max(...masses.map((m) => Math.abs(m - ref.mass)));
    }
    const legmCdg = legMass(p, geo.L2, geo.L3, mCdg[0], mCdg[1], mCdg[2]);
    const mMin = Math.min(...masses);
    Mmin = Math.min(body.walk[0], body.manip[0]) + 4 * legMass(p, geo.L2, geo.L3, mMin, mMin, mMin);
    const offs = mode === "exact" ? opts.servo_offsets : null;
    const acc = tk.accel / G;

    // ---- passe 1 : CdG de reference + borne eps_legs
    const states = [];
    phases.forEach((ph, pi) => ph.states.forEach((st) => states.push([pi, st])));
    const Gref = [], Mref = [], poseCache = [];
    let Exy = 0, Ez = 0;
    for (const [pi, st] of states) {
      const [mb, gb] = body[st.cdg];
      const M = mb + 4 * legmCdg;
      const Gs = new Array(NS), cache = new Array(NS);
      const legsList = st.ground.map((g) => g).concat(st.swing ? [st.swing] : []);
      for (let s = 0; s < NS; s++) {
        const mom = [0, 0, 0], sP = { C1: [0, 0, 0], C2: [0, 0, 0], knee: [0, 0, 0] };
        const legsC = {};
        for (const [l, off, zf] of legsList) {
          const ps = legPose(p, geo, l, off, zf, st.yaw, sw[s]);
          legsC[l] = ps;
          const mo = legMoment(p, ps, mCdg[0], mCdg[1], mCdg[2], offs);
          for (let k = 0; k < 3; k++) { mom[k] += mo[k]; sP.C1[k] += ps.C1[k]; sP.C2[k] += ps.C2[k]; sP.knee[k] += ps.knee[k]; }
        }
        const gabs = [gb[0], gb[1], geo.h + gb[2]];
        const Gr = [0, 1, 2].map((k) => (mb * gabs[k] + mom[k]) / M);
        for (const t of ["C1", "C2", "knee"]) {
          const dx = sP[t][0] - 4 * Gr[0], dy = sP[t][1] - 4 * Gr[1], dz = sP[t][2] - 4 * Gr[2];
          Exy = Math.max(Exy, Math.hypot(dx, dy)); Ez = Math.max(Ez, Math.abs(dz));
        }
        Gs[s] = Gr; cache[s] = legsC;
      }
      Gref.push(Gs); Mref.push(M); poseCache.push(cache);
    }
    const epsLegsXY = (3 * dm * Exy) / Mmin, epsLegsZ = (3 * dm * Ez) / Mmin;

    // ---- passe 2
    const nP = phases.length;
    const mk = (v) => Array.from({ length: nP }, () => new Array(NS).fill(v));
    const geoOk = mk(true), nesm = mk(Infinity), ssmc = mk(Infinity), swA2 = mk(0), swB2 = mk(0), swA3 = mk(0);
    const kf = Array.from({ length: nP }, () => Array.from({ length: NS }, () => [0, 0, 0]));
    const angMin = Array.from({ length: nP }, () => Array.from({ length: NS }, () => [0, 1, 2, 3].map(() => [Infinity, Infinity, Infinity])));
    const angMax = Array.from({ length: nP }, () => Array.from({ length: NS }, () => [0, 1, 2, 3].map(() => [-Infinity, -Infinity, -Infinity])));
    // diagnostic des causes de rejet geometrique (par phase et balancement)
    const why = Array.from({ length: nP }, () => new Int32Array(NS));
    const lamKg = lg.seg_lin_mass / 1000;
    const detail = Array.from({ length: nP }, () => []);   // par etat : donnees pour l'affichage
    states.forEach(([pi, st], si) => {
      const [mb, gb, hb] = body[st.cdg];
      const M = Mref[si];
      const hxy = [hb[0] * (mb / M) + eps[0] + epsLegsXY, hb[1] * (mb / M) + eps[1] + epsLegsXY];
      const hz = hb[2] * (mb / M) + eps[2] + epsLegsZ;
      const ground = st.ground.map((g) => g[0]);
      const order = CYCLIC.filter((l) => ground.includes(l));
      const zmap = new Map(st.ground.map((g) => [g[0], g[2]]));
      const zf = order.map((l) => zmap.get(l));
      const ptp = Math.max(...zf) - Math.min(...zf);
      const stDetail = { si, label: st.label, yaw: st.yaw, st, ground, order, zf, swing: st.swing ? st.swing[0] : null, hxy, hz, cdg: st.cdg };
      for (let s = 0; s < NS; s++) {
        const legs = poseCache[si][s];
        const Gr = Gref[si][s];
        const Gc = [Gr[0], Gr[1]];
        const zmax = Gr[2] + hz;
        const feet = order.map((l) => legs[l].rel);
        const pm = polygonMargins(feet, Gc, hxy, zmax);
        let nes = pm.nesm;
        if (ptp > 0) {
          const f3 = feet.map((f, j) => [f[0], f[1], zf[j]]);
          let n3 = Infinity;
          for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1])
            n3 = Math.min(n3, nesm3d(f3, [Gc[0] + sx * hxy[0], Gc[1] + sy * hxy[1], Gr[2] + sz * hz]));
          nes = n3;
        }
        let lamLeg = {};
        if (order.length === 3) {
          const { lam, grad } = barycentric(Gc, feet[0], feet[1], feet[2]);
          order.forEach((l, j) => { lamLeg[l] = clip(boxMax(lam[j], grad[j], hxy), 0, 1); });
        } else {
          const f4 = [0, 1, 2, 3].map((l) => legs[l].rel);
          const l4 = fourFeetMaxLambda(f4, Gc, hxy);
          [0, 1, 2, 3].forEach((l) => { lamLeg[l] = l4[l]; });
        }
        const kk = [0, 0, 0];
        for (const l of ground) {
          const ps = legs[l];
          kk[0] = Math.max(kk[0], lamLeg[l] * acc * ps.rh);
          kk[1] = Math.max(kk[1], lamLeg[l] * ps.D);
          kk[2] = Math.max(kk[2], lamLeg[l] * ps.L3 * Math.abs(Math.cos(ps.be)));
        }
        let ok = true; let w = 0;
        for (const ls of Object.keys(legs)) {
          const l = +ls, ps = legs[l];
          if (!ps.reachMax) { ok = false; w |= 1; }
          if (!ps.reachMin) { ok = false; w |= 2; }
          if (!ps.Dpos) { ok = false; w |= 4; }
          if (!(ps.th3 >= mg.theta3_min && ps.th3 <= mg.theta3_max)) { ok = false; w |= 8; }
          const cl = mg.chassis_clearance;
          if (!(Math.abs(ps.rel[0]) >= ch.N_ch / 2 + cl || Math.abs(ps.rel[1]) >= ch.M_ch / 2 + cl)) { ok = false; w |= 16; }
          if (ch.mount === "dedans") {
            const o = ch.overhang_C2;
            if (!(Math.abs(ps.C2[0]) <= ch.N_ch / 2 + o && Math.abs(ps.C2[1]) <= ch.M_ch / 2 + o)) { ok = false; w |= 32; }
          }
          const am = angMin[pi][s][l], aM = angMax[pi][s][l];
          [ps.th1, ps.th2, ps.th3].forEach((v, j) => { am[j] = Math.min(am[j], v); aM[j] = Math.max(aM[j], v); });
        }
        for (const [la, lb] of [[0, 2], [1, 3]]) {
          const pa = legs[la], pb = legs[lb];
          const ds = segSegDist([pa.C1[0], pa.C1[1]], pa.rel, [pb.C1[0], pb.C1[1]], pb.rel);
          if (!(ds >= mg.feet_clearance)) { ok = false; w |= 64; }
        }
        if (st.swing) {
          const ps = legs[st.swing[0]];
          const xk = ps.L2 * Math.cos(ps.al), xf = ps.D;
          swA2[pi][s] = Math.max(swA2[pi][s], (G / 1000) * Math.abs(lamKg * ps.L2 * xk / 2 + lg.bracket_mass * xk + lamKg * ps.L3 * (xk + xf) / 2 + lg.foot_mass * xf));
          swB2[pi][s] = Math.max(swB2[pi][s], (G / 1000) * Math.abs(xk));
          swA3[pi][s] = Math.max(swA3[pi][s], (G / 1000) * Math.abs((lamKg * ps.L3 / 2 + lg.foot_mass) * (xf - xk)));
        }
        why[pi][s] |= w;
        geoOk[pi][s] = geoOk[pi][s] && ok;
        nesm[pi][s] = Math.min(nesm[pi][s], nes);
        ssmc[pi][s] = Math.min(ssmc[pi][s], pm.ssm - zmax * acc - mg.eps_foot);
        for (let j = 0; j < 3; j++) kf[pi][s][j] = Math.max(kf[pi][s][j], kk[j]);
      }
      detail[pi].push(stDetail);
    });

    const res = { geo, pre, phases, sw, body, eps, epsLegsXY, epsLegsZ, Gref, geoOk, nesm, ssmc, k: kf, swA2, swB2, swA3, angMin, angMax, why, detail, mBodyMax };
    if (combo) res.servo = resolve(p, res, combo);
    return res;
  }

  // ---------------------------------------------- servos (comme _resolve_servos)
  function resolve(p, r, combo) {
    const S = servoList(p), mg = p.margins, b = p.body;
    const [s1, s2, s3] = combo.map((i) => S[i]);
    const SF = mg.SF, nmin = mg.nesm_min, smin = mg.s_min ?? 0;
    const geo = r.geo, nP = r.phases.length, NS = r.sw.length;
    const legm = legMass(p, geo.L2, geo.L3, s1.mass, s2.mass, s3.mass);
    const Mc = r.mBodyMax + 4 * legm;
    const Cu = [s1.C_cont / SF, s2.C_cont / SF, s3.C_cont / SF];
    const Cst = [s1.C_stall, s2.C_stall, s3.C_stall];
    const out = { names: [s1.name, s2.name, s3.name], legm, Mc, Cu, Cst, phases: [] };
    let f2 = Infinity, f3 = -Infinity, allPh = true, critical = null, critVal = Infinity;
    const tauMaxBest = [0, 0, 0], tmin = [-Infinity, -Infinity, -Infinity];
    const spanMin = [0, 1, 2, 3].map(() => [Infinity, Infinity, Infinity]);
    const spanMax = [0, 1, 2, 3].map(() => [-Infinity, -Infinity, -Infinity]);
    for (let pi = 0; pi < nP; pi++) {
      let best = -1, bestVal = -Infinity, t2min = Infinity;
      const tminPh = [Infinity, Infinity, Infinity];
      const cnt = { geo: 0, stab: 0, couple: 0, ok: 0 };
      for (let s = 0; s < NS; s++) {
        const tau = r.k[pi][s].map((kj) => (Mc * G * kj) / 1000);
        const stab = r.geoOk[pi][s] && r.nesm[pi][s] >= nmin && r.ssmc[pi][s] >= smin;
        const swC2 = r.swA2[pi][s] + r.swB2[pi][s] * s3.mass;
        const tq = tau[0] <= Cu[0] && tau[1] <= Cu[1] && tau[2] <= Cu[2] && swC2 <= Cu[1] && r.swA3[pi][s] <= Cu[2];
        const tq13 = tau[0] <= Cu[0] && tau[2] <= Cu[2] && r.swA3[pi][s] <= Cu[2];
        if (!r.geoOk[pi][s]) cnt.geo++; else if (!stab) cnt.stab++; else if (!tq) cnt.couple++; else cnt.ok++;
        if (stab && tq13) t2min = Math.min(t2min, Math.max(tau[1], swC2));
        const feas = stab && tq;
        if (feas) {
          for (let j = 0; j < 3; j++) tminPh[j] = Math.min(tminPh[j], tau[j]);
          if (r.nesm[pi][s] > bestVal) { bestVal = r.nesm[pi][s]; best = s; }
        }
      }
      const ok = best >= 0;
      f3 = Math.max(f3, t2min);
      for (let j = 0; j < 3; j++) tmin[j] = Math.max(tmin[j], tminPh[j]);
      const ph = { name: r.phases[pi].name, ok, best, cnt, t2req: t2min };
      if (ok) {
        const tau = r.k[pi][best].map((kj) => (Mc * G * kj) / 1000);
        ph.tau = tau; ph.nesm = r.nesm[pi][best]; ph.ssmc = r.ssmc[pi][best]; ph.sway = r.sw[best];
        ph.swC2 = r.swA2[pi][best] + r.swB2[pi][best] * s3.mass; ph.swC3 = r.swA3[pi][best];
        for (let j = 0; j < 3; j++) tauMaxBest[j] = Math.max(tauMaxBest[j], tau[j]);
        if (ph.nesm < critVal) { critVal = ph.nesm; critical = ph.name; }
        f2 = Math.min(f2, ph.nesm);
        for (let l = 0; l < 4; l++) for (let j = 0; j < 3; j++) {
          spanMin[l][j] = Math.min(spanMin[l][j], r.angMin[pi][best][l][j]);
          spanMax[l][j] = Math.max(spanMax[l][j], r.angMax[pi][best][l][j]);
        }
      } else allPh = false;
      out.phases.push(ph);
    }
    const span = [0, 1, 2].map((j) => Math.max(...[0, 1, 2, 3].map((l) => spanMax[l][j] - spanMin[l][j])));
    const rangeAvail = [s1.range, s2.range, s3.range].map((v) => v - 2 * mg.joint_margin);
    const spanOk = span.every((v, j) => v <= rangeAvail[j]);
    const mNoLimb = b.chassis.m + b.battery.m;
    const massRobot = mNoLimb + 4 * legm;
    const massOk = massRobot <= b.mass_robot_max_sans_bras;
    Object.assign(out, {
      f1: 4 * legm, f2: allPh ? f2 : NaN, f3, tauMaxBest, tmin, span, rangeAvail, spanOk, massRobot, massOk,
      critical, allPh, admissible: r.pre.ok && allPh && spanOk && massOk,
      u: tauMaxBest.map((t, j) => t / [s1, s2, s3][j].C_cont),
      fracBloc: [tmin[0] / Cst[0], f3 / Cst[1], tmin[2] / Cst[2]],
    });
    return out;
  }

  /* Recalcule, pour l'affichage, tout ce qui concerne un etat (pi, j) a la
   * position du corps d'indice s : poses des 4 pattes, CdG, polygone, marges,
   * parts de poids. Memes formules que evaluate(). */
  function stateView(p, r, pi, j, s) {
    const d = r.detail[pi][j], st = d.st, geo = r.geo, sw = r.sw[s];
    const legs = {};
    for (const [l, off, z] of st.ground) legs[l] = legPose(p, geo, l, off, z, st.yaw, sw);
    if (st.swing) { const [l, off, z] = st.swing; legs[l] = legPose(p, geo, l, off, z, st.yaw, sw); }
    const Gr = r.Gref[d.si][s], Gc = [Gr[0], Gr[1]], zmax = Gr[2] + d.hz;
    const feet = d.order.map((l) => legs[l].rel);
    const pm = polygonMargins(feet, Gc, d.hxy, zmax);
    let lamLeg = {};
    if (d.order.length === 3) {
      const { lam, grad } = barycentric(Gc, feet[0], feet[1], feet[2]);
      d.order.forEach((l, q) => { lamLeg[l] = clip(boxMax(lam[q], grad[q], d.hxy), 0, 1); });
    } else {
      const l4 = fourFeetMaxLambda([0, 1, 2, 3].map((l) => legs[l].rel), Gc, d.hxy);
      [0, 1, 2, 3].forEach((l) => { lamLeg[l] = l4[l]; });
    }
    return { legs, Gr, Gc, zmax, feet, order: d.order, zf: d.zf, ssm: pm.ssm, edges: pm.edges, lamLeg, hxy: d.hxy, hz: d.hz, sway: sw, label: d.label, swing: d.swing };
  }

  const WHY = ["portée max", "portée min (repli)", "pied derrière C2", "θ3 hors limites", "pied sous le châssis", "axe C2 hors châssis", "pattes trop proches"];
  const api = { WHY, stateView, G, LEG_NAMES, LEG_SIGNS, CYCLIC, servoList, combos, legMass, armMass, armPoints, bodyCdg, bodyNominal, bodyBox, ik, geometry, prefilter, buildPhases, barycentric, polygonMargins, nesm3d, fourFeetMaxLambda, segSegDist, legPose, legMoment, evaluate, resolve };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.Gr2Model = api;
})(typeof self !== "undefined" ? self : this);
