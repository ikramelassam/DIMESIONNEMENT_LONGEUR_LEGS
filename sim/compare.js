/* Gr2 - lecture "probleme inverse" : on donne les variables, on lit
 *   (1) chaque contrainte avec sa valeur, sa limite et sa marge,
 *   (2) les trois fonctions objectifs (f1 masse, f2 NESM, f3 couple C2),
 *   (3) la position du design par rapport a l'optimum (front de Pareto 3D de reference).
 * Fonctions pures, utilisables dans le navigateur et sous node (sim/experiences.js).
 * Conventions : f1 et f3 a MINIMISER, f2 a MAXIMISER (comme gr2/explore.py). */
(function (root) {
  "use strict";
  const fin = (v) => typeof v === "number" && isFinite(v);

  /* Contraintes chiffrees. M = Gr2Model, p = parametres, r = resultat de M.evaluate (avec r.servo). */
  function constraints(M, p, r) {
    const s = r.servo, g = r.geo, tk = p.task, mg = p.margins, ch = p.chassis, b = p.body;
    const gab = p.gabarit ?? 400, sp = mg.servo_spacing ?? 40;
    const rows = [];
    const add = (id, grp, label, val, lim, sense, unit, hard, why) => {
      const margin = !fin(val) ? -Infinity : sense === "<=" ? lim - val : val - lim;
      // valeur non finie sur une grandeur qui dépend d'une phase réalisable : conséquence de PH, pas une cause
      const undef = !fin(val) && (id[0] === "R" || id === "NESM" || id === "NESM*" || id[0] === "T" || id[0] === "U") && !s.phases.every((q) => q.ok);
      rows.push({ id, grp, label, val, lim, sense, unit, hard: hard !== false, margin, ok: margin >= 0, undef, why });
    };
    const hmin = tk.obstacle + tk.h_margin;
    add("G5", "posture", "garde au sol h", g.h, hmin, ">=", "mm", true, "le corps doit passer l'obstacle de 30 mm avec une marge");
    const th3 = r.pre.th3;
    rows.push({ id: "G3", grp: "posture", label: "θ₃ = α + β au repos", val: th3, lim: `[${mg.theta3_min} ; ${mg.theta3_max}]`, sense: "in", unit: "°", hard: true,
      margin: Math.min(th3 - mg.theta3_min, mg.theta3_max - th3), ok: th3 >= mg.theta3_min && th3 <= mg.theta3_max, why: "angle relatif genou hors de la plage réalisable" });
    add("G6", "posture", "empreinte au sol (max des deux côtés)", Math.max(r.pre.ext[0], r.pre.ext[1]), gab, "<=", "mm", true, "gabarit 400 × 400 mm du cahier des charges");
    if ((p.gabarit_min ?? 0) > 0) add("G6b", "posture", "empreinte au sol imposée (max des deux côtés)", Math.max(r.pre.ext[0], r.pre.ext[1]), p.gabarit_min, ">=", "mm", true, "empreinte minimale imposée par l'équipe (0 = libre)");
    add("G10", "posture", "châssis ≤ gabarit", Math.max(ch.N_ch, ch.M_ch), gab, "<=", "mm", true, "dimension du châssis");
    const dx = g.C1[0][0] - g.C1[2][0], dy = g.C1[0][1] - g.C1[1][1];
    add("G7", "posture", "écart des axes C1", Math.min(dx, dy), sp, ">=", "mm", true, "encombrement des servos de lacet");
    const nOk = s.phases.filter((q) => q.ok).length;
    add("PH", "phases", "phases réalisables (sur " + s.phases.length + ")", nOk, s.phases.length, ">=", "", true,
      "pour chaque phase il faut une position du corps qui respecte géométrie, stabilité et couples");
    add("NESM", "stabilité", "NESM garanti f₂ (pire phase)", s.f2, mg.nesm_min, ">=", "mm", true, "marge d'énergie de basculement au pire CdG");
    add("NESM*", "stabilité", "NESM cible de l'équipe", s.f2, mg.nesm_cible ?? 2, ">=", "mm", false, "cible souple (ε-contrainte), pas une exigence dure");
    const ob = objectives(r, p);
    for (let j = 0; j < 3; j++) {
      let req = j === 1 ? s.f3 : s.tmin[j], free = false;
      if (!fin(req) && ob.req_free) { req = ob.req_free[j]; free = true; }   // sans position admissible : couple requis si le servo était illimité
      add("T" + (j + 1), "couples", `couple requis C${j + 1} (${s.names[j]})${free ? " — si le servo était illimité" : ""}`, req, s.Cu[j], "<=", "N·m", true,
        "couple minimal sur toutes les positions admissibles, au pire de toutes les phases ; limite = frac × C_blocage / SF");
      if (free) { const row = rows[rows.length - 1]; row.free = true; row.undef = row.ok; }   // requis <= limite : le défaut vient d'ailleurs
    }
    for (let j = 0; j < 3; j++) add("R" + (j + 1), "courses", `course utilisée C${j + 1}`, s.span[j], s.rangeAvail[j], "<=", "°", true, "course du servo − 2 × marge d'articulation");
    add("M", "masse", "robot sans bras", s.massRobot, b.mass_robot_max_sans_bras, "<=", "kg", true, "budget de masse");
    s.fracBloc.forEach((f, j) => add("U" + (j + 1), "recommandation", `part du couple de blocage utilisée C${j + 1}`, f, 0.2, "<=", "", false, "Robotis conseille ≤ 1/5 du couple de blocage (non respecté par la plupart des designs)"));
    return rows;
  }

  /* Objectifs. f2partial : NESM minimal sur les seules phases réalisables (indicatif si le design est non admissible). */
  function objectives(r, p) {
    const s = r.servo, okPh = s.phases.filter((q) => q.ok);
    const f2p = okPh.length ? Math.min(...okPh.map((q) => q.nesm)) : NaN;
    // valeurs "sans limite de couple des servos" : seules la géométrie et la stabilité restent imposées.
    // Dit ce que valent f2 et f3 si le seul problème est un servo trop faible.
    const mg = p.margins, nmin = mg.nesm_min, smin = mg.s_min ?? 0, G = 9.81, nP = r.geoOk.length;
    let f2n = Infinity, f3n = -Infinity, okAll = true; const free = [-Infinity, -Infinity, -Infinity];
    const m3 = s.names ? p.servos.find((z) => z.name === s.names[2]).mass : 0;
    for (let pi = 0; pi < nP; pi++) {
      let best = -Infinity; const tq = [Infinity, Infinity, Infinity];
      for (let q = 0; q < r.sw.length; q++) {
        if (!(r.geoOk[pi][q] && r.nesm[pi][q] >= nmin && r.ssmc[pi][q] >= smin)) continue;
        best = Math.max(best, r.nesm[pi][q]);
        const tau = r.k[pi][q].map((kj) => (s.Mc * G * kj) / 1000);
        tq[0] = Math.min(tq[0], tau[0]);
        tq[1] = Math.min(tq[1], Math.max(tau[1], r.swA2[pi][q] + r.swB2[pi][q] * m3));
        tq[2] = Math.min(tq[2], Math.max(tau[2], r.swA3[pi][q]));
      }
      if (best === -Infinity) { okAll = false; continue; }
      f2n = Math.min(f2n, best); f3n = Math.max(f3n, tq[1]);
      for (let j = 0; j < 3; j++) free[j] = Math.max(free[j], tq[j]);
    }
    return { f1: s.f1, f2: s.f2, f3: s.f3, f2partial: f2p, f2_notq: okAll ? f2n : NaN, f3_notq: okAll ? f3n : NaN, req_free: okAll ? free : null,
      admissible: !!s.admissible, nOk: okPh.length, nPh: s.phases.length, critical: s.critical };
  }

  const FK = ["f1_masse_pattes", "f2_nesm", "f3_couple_C2"];
  const VARS = ["L2", "L3", "alpha", "beta", "xi"];
  const TOL = 1e-7;               // tolérance relative : deux valeurs à 1e-7 près sont égales
  const lt = (a, b) => a < b - TOL * Math.max(1, Math.abs(b)), le = (a, b) => a <= b + TOL * Math.max(1, Math.abs(b));
  function dom(a, b) {            // a domine b : pas pire partout (à TOL près), strictement meilleur quelque part
    return le(a[0], b[0]) && le(b[1], a[1]) && le(a[2], b[2]) && (lt(a[0], b[0]) || lt(b[1], a[1]) || lt(a[2], b[2]));
  }

  /* Comparaison à une référence : ref = { front: [rows], rob: [rows], choix: {coude, eps_contrainte, couple_min} }.
   * f = [f1, f2, f3] du design (valeurs valides si admissible). */
  function compare(ref, f, x, bounds) {
    const fr = ref.front && ref.front.length ? ref.front : ref.rob || [];
    if (!fr.length) return null;
    const F = fr.map((r) => FK.map((k) => r[k]));
    const ideal = [Math.min(...F.map((q) => q[0])), Math.max(...F.map((q) => q[1])), Math.min(...F.map((q) => q[2]))];
    const nadir = [Math.max(...F.map((q) => q[0])), Math.min(...F.map((q) => q[1])), Math.max(...F.map((q) => q[2]))];
    const sc = [nadir[0] - ideal[0] || 1, ideal[1] - nadir[1] || 1, nadir[2] - ideal[2] || 1];
    const regret = (q) => Math.sqrt(((q[0] - ideal[0]) / sc[0]) ** 2 + ((ideal[1] - q[1]) / sc[1]) ** 2 + ((q[2] - ideal[2]) / sc[2]) ** 2);
    const valid = f.every(fin);
    const out = { ideal, nadir, valid, regret: valid ? regret(f) : Infinity, n_ref: fr.length };
    if (valid) {
      const dm = F.map((q, i) => [q, i]).filter(([q]) => dom(q, f));
      out.dominators = dm.length;
      out.dominated_by = dm.slice().sort((a, b) => regret(a[0]) - regret(b[0])).slice(0, 3).map(([q, i]) => ({ i, f: q, row: fr[i] }));
      out.on_front = dm.length === 0 && !F.some((q) => q[0] === f[0] && q[1] === f[1] && q[2] === f[2]) ? "non dominé par le front de référence" : dm.length ? "dominé" : "sur le front";
    }
    // points de référence nommés
    const named = [];
    const ch = ref.choix || {};
    const lab = { coude: "Compromis (coude)", eps_contrainte: "C₂ minimal avec NESM ≥ cible", couple_min: "Plus petit C₂ du front robuste" };
    for (const k of ["coude", "eps_contrainte", "couple_min"]) if (ch[k]) named.push({ key: k, label: lab[k], row: ch[k] });
    const iMin1 = F.reduce((b, q, i) => (q[0] < F[b][0] ? i : b), 0), iMax2 = F.reduce((b, q, i) => (q[1] > F[b][1] ? i : b), 0), iMin3 = F.reduce((b, q, i) => (q[2] < F[b][2] ? i : b), 0);
    named.push({ key: "min_f1", label: "Le plus léger du front", row: fr[iMin1] }, { key: "max_f2", label: "Le plus stable du front", row: fr[iMax2] }, { key: "min_f3", label: "Couple C₂ minimal du front", row: fr[iMin3] });
    out.named = named.map((n) => {
      const q = FK.map((k) => n.row[k]);
      const d = valid ? [f[0] - q[0], f[1] - q[1], f[2] - q[2]] : [NaN, NaN, NaN];
      const rel = valid ? [d[0] / q[0], d[1] / q[1], d[2] / q[2]] : [NaN, NaN, NaN];
      const rel_ = rel;
      let rapport = "—";
      if (valid) rapport = FK.every((_, i) => !lt(q[i], f[i]) && !lt(f[i], q[i])) ? "identique" : dom(q, f) ? "le point de référence domine" : dom(f, q) ? "le design domine" : "incomparable";
      return { key: n.key, label: n.label, row: n.row, f: q, d, rel: rel_, regret: regret(q), rapport };
    });
    // point du front le plus proche dans l'espace des variables (normalisé par les bornes)
    if (x && bounds) {
      let best = -1, bd = Infinity;
      fr.forEach((r, i) => {
        const dd = Math.sqrt(VARS.reduce((a, k, j) => a + ((x[k] - r[k]) / (bounds.hi[j] - bounds.lo[j])) ** 2, 0));
        if (dd < bd) { bd = dd; best = i; }
      });
      out.nearest = { row: fr[best], dist: bd };
    }
    return out;
  }

  /* Différences entre les paramètres courants et ceux de la référence (chemins "a.b.c"). */
  function diffParams(cur, ref, ignore) {
    const out = [];
    const walk = (a, b, path) => {
      if (ignore && ignore.some((q) => path === q || path.startsWith(q + "."))) return;
      if (a && b && typeof a === "object" && typeof b === "object" && !Array.isArray(a) && !Array.isArray(b)) {
        for (const k of new Set([...Object.keys(a), ...Object.keys(b)])) walk(a[k], b[k], path ? path + "." + k : k);
      } else if (JSON.stringify(a) !== JSON.stringify(b)) out.push([path, b, a]);
    };
    walk(cur, ref, "");
    return out;
  }

  const api = { constraints, objectives, compare, diffParams, dom, FK, VARS };
  if (typeof module !== "undefined" && module.exports) module.exports = api; else root.Gr2Cmp = api;
})(typeof self !== "undefined" ? self : this);
